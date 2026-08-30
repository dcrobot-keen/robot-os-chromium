// LocalizationNode — map-relative pose from scan matching (roadmap.md
// Phase 9, step 5).
//
// Runs the correlative scan matcher (scan-matcher.js) against a static map
// (from an iPhone-scan slicemap, wall cells only) every laser scan, and
// publishes the result on `correctionTopic` -- the exact topic
// PoseFusionNode fuses a VPS-style absolute fix on. So on a TurtleBot3 with
// no VPS, this IS the absolute-fix source: OdometryNode dead-reckons
// continuously, LocalizationNode pulls it back onto the prebuilt map.
//
// Between scans the estimate is carried forward by the odometry delta (the
// "map -> odom" transform stays fixed; the pose moves with odom). A run of
// low-scoring matches -> "lost": stop publishing (PoseFusion coasts on
// odom) and widen the search until a confident match relocalizes. setPose()
// is the "2D Pose Estimate" hook -- a click on the map seeds the estimate.

import { matchScan } from './scan-matcher.js';

function wrap(a) {
  return Math.atan2(Math.sin(a), Math.cos(a));
}

// Move `base` (map frame) by the motion odom made from `from` to `to`.
function applyOdomDelta(base, from, to) {
  const dx = to.x - from.x;
  const dy = to.y - from.y;
  const rot = base.theta - from.theta; // odom-frame translation -> map frame
  const c = Math.cos(rot);
  const s = Math.sin(rot);
  return {
    x: base.x + c * dx - s * dy,
    y: base.y + s * dx + c * dy,
    theta: wrap(base.theta + wrap(to.theta - from.theta)),
  };
}

export class LocalizationNode {
  /**
   * @param {object} bus - LocalBus
   * @param {object} opts
   * @param {string} opts.scanTopic
   * @param {string} opts.odomTopic - OdometryNode's dead-reckoned pose (prior + carry-forward)
   * @param {string} opts.correctionTopic - {x,y,theta} published here for PoseFusionNode
   * @param {{grid,field,sigmaM}} opts.likelihoodField - from buildLikelihoodField (wall cells)
   * @param {{x,y,theta}} [opts.initialPose] - map-frame starting guess (else waits for setPose)
   * @param {object} [opts.matchOpts] - passed to matchScan
   * @param {number} [opts.minScore=0.35] - tracking matches below this are not trusted
   * @param {number} [opts.relocalizeMinScore=0.6] - stricter bar to *re-acquire*
   *   after "lost": the widened search finds spurious high-scoring poses far
   *   more readily than the tight tracking window, so a fluke shouldn't count.
   * @param {number} [opts.lostAfter=5] - consecutive low matches before going "lost"
   * @param {number} [opts.relocalizeWindowM=0.8] - widened search after "lost";
   *   this recovers tracking that slipped, not a kidnapped robot (a full global
   *   relocalization would multi-start over the whole map -- future work).
   * @param {number} [opts.relocalizeWindowRad=Math.PI/3]
   * @param {(e:object)=>void} [opts.onEvent] - {type:'match'|'weak'|'lost'|'relocalized'|'set-pose', score, pose}
   */
  constructor(bus, {
    scanTopic, odomTopic, correctionTopic, likelihoodField,
    initialPose = null, matchOpts = {},
    minScore = 0.35, relocalizeMinScore = 0.6, lostAfter = 5,
    relocalizeWindowM = 0.8, relocalizeWindowRad = Math.PI / 3,
    onEvent,
  } = {}) {
    if (!scanTopic || !odomTopic || !correctionTopic || !likelihoodField) {
      throw new Error('LocalizationNode requires scanTopic, odomTopic, correctionTopic, likelihoodField');
    }
    this._bus = bus;
    this._correctionTopic = correctionTopic;
    this._lf = likelihoodField;
    this._matchOpts = matchOpts;
    this._minScore = minScore;
    this._relocalizeMinScore = relocalizeMinScore;
    this._lostAfter = lostAfter;
    this._reloc = { windowM: relocalizeWindowM, windowRad: relocalizeWindowRad };
    this._onEvent = onEvent;

    this._mapPose = initialPose ? { ...initialPose } : null;
    this._lastOdom = null;      // most recent odom message
    this._odomAtCarry = null;   // odom pose the current _mapPose was last carried from
    this._lowStreak = 0;
    this._lost = false;
    this._lastScore = 0;

    this._unsubOdom = bus.subscribe(odomTopic, (p) => this._onOdom(p));
    this._unsubScan = bus.subscribe(scanTopic, (s) => this._onScan(s));
  }

  /** Seed the estimate (a click on the map). Clears "lost". */
  setPose(pose) {
    this._mapPose = { x: pose.x, y: pose.y, theta: pose.theta };
    this._odomAtCarry = this._lastOdom ? { ...this._lastOdom } : null;
    this._lowStreak = 0;
    this._lost = false;
    this._emit('set-pose', 1, this._mapPose);
  }

  getPose() {
    return this._mapPose ? { ...this._mapPose } : null;
  }

  _onOdom(pose) {
    // carry the map-frame estimate forward by whatever odom moved
    if (this._mapPose && this._odomAtCarry) {
      this._mapPose = applyOdomDelta(this._mapPose, this._odomAtCarry, pose);
    }
    this._odomAtCarry = { ...pose };
    this._lastOdom = { ...pose };
  }

  _onScan(scan) {
    if (!this._mapPose || !scan || !Array.isArray(scan.ranges)) return;

    const opts = this._lost
      ? { ...this._matchOpts, windowM: this._reloc.windowM, windowRad: this._reloc.windowRad }
      : this._matchOpts;
    const { pose, score, evals } = matchScan(this._lf, this._mapPose, scan, opts);
    this._lastScore = score;

    const bar = this._lost ? this._relocalizeMinScore : this._minScore;
    if (score >= bar) {
      const wasLost = this._lost;
      this._mapPose = pose;
      this._odomAtCarry = this._lastOdom ? { ...this._lastOdom } : this._odomAtCarry;
      this._lowStreak = 0;
      this._lost = false;
      this._bus.publish(this._correctionTopic, { x: pose.x, y: pose.y, theta: pose.theta });
      this._emit(wasLost ? 'relocalized' : 'match', score, pose, evals);
    } else {
      this._lowStreak++;
      this._emit('weak', score, this._mapPose, evals);
      if (!this._lost && this._lowStreak >= this._lostAfter) {
        this._lost = true;
        this._emit('lost', score, this._mapPose, evals);
      }
      // no correction published -> PoseFusionNode coasts on odometry
    }
  }

  _emit(type, score, pose, evals) {
    if (this._onEvent) this._onEvent({ type, score, pose: pose && { ...pose }, evals, lost: this._lost });
  }

  stop() {
    this._unsubOdom();
    this._unsubScan();
  }
}
