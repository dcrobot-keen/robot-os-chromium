// PathFollowerNode — roadmap.md Phase 7's last remaining piece: "path + pose
// -> cmd_vel (pure pursuit 수준으로 시작)". Subscribes to a path topic (a
// list of [x,y] waypoints, e.g. from pathfinder's Go planner) and a pose
// topic (ground-truth from the simulator today; a real OdometyNode's fused
// pose later -- this node doesn't care which), and publishes {left, right}
// on the same drive/cmd_vel topic TeleopNode already uses, so no new
// consumer is needed on the transport side.
//
// Deliberately dumb: it re-targets a lookahead point on the *given* path
// every tick and never re-plans or dodges anything new -- exactly the
// "no live replanning/avoidance" scope roadmap.md calls out. Obstacle
// avoidance is pathfinder's planner's job before the path ever gets here.

// --- pure pursuit math, exported standalone for testing without a bus ---

/** World-frame point -> robot-local frame (x forward, y left). */
export function toRobotFrame(point, pose) {
  const dx = point[0] - pose.x;
  const dy = point[1] - pose.y;
  const cos = Math.cos(pose.theta);
  const sin = Math.sin(pose.theta);
  return { x: cos * dx + sin * dy, y: -sin * dx + cos * dy };
}

/** First path point at least `lookaheadM` away from pose, scanning forward
 * from `fromIndex` -- falls back to the final point if the path ends first
 * (so the robot heads straight at the goal on the last leg). Returns
 * { point, index } so the caller can avoid re-scanning already-passed points. */
export function findLookaheadPoint(path, pose, lookaheadM, fromIndex = 0) {
  for (let i = fromIndex; i < path.length; i++) {
    if (Math.hypot(path[i][0] - pose.x, path[i][1] - pose.y) >= lookaheadM) {
      return { point: path[i], index: i };
    }
  }
  return { point: path[path.length - 1], index: path.length - 1 };
}

/**
 * One pure-pursuit control step.
 * @param {number[][]} path - [[x,y], ...] in the same frame as pose
 * @param {{x:number,y:number,theta:number}} pose
 * @param {{wheelRadius:number, wheelSeparation:number, maxWheelRpm:number}} geometry
 * @param {{lookaheadM?:number, cruiseMps?:number, goalToleranceM?:number, startIndex?:number, turnInPlaceRad?:number, turnGain?:number}} [options]
 * @returns {{left:number, right:number, atGoal:boolean, lookaheadIndex:number}}
 *   left/right are normalized [-1,1], ready for createDriveDevice.setVelocity.
 */
export function pursuitStep(path, pose, geometry, options = {}) {
  const {
    lookaheadM = 0.3, cruiseMps = 0.12, goalToleranceM = 0.1, startIndex = 0,
    turnInPlaceRad = 1.0, turnGain = 0.35,
  } = options;

  const goal = path[path.length - 1];
  const distToGoal = Math.hypot(goal[0] - pose.x, goal[1] - pose.y);
  if (distToGoal <= goalToleranceM) {
    return { left: 0, right: 0, atGoal: true, lookaheadIndex: path.length - 1 };
  }

  let { point: lookahead, index } = findLookaheadPoint(path, pose, lookaheadM, startIndex);
  // Final approach: inside one lookahead of the goal aim straight at the goal
  // and slow down in proportion to the remaining distance. Without this the
  // robot arrives at cruise speed, overshoots the tolerance circle, and orbits
  // the goal until the (noisy) pose estimate happens to fall inside it -- seen
  // as a robot "circling for a minute before stopping" on the real map.
  let v = cruiseMps;
  if (distToGoal < lookaheadM) {
    lookahead = goal;
    index = path.length - 1;
    v = cruiseMps * Math.max(0.3, distToGoal / lookaheadM);
  }
  const local = toRobotFrame(lookahead, pose);

  // Pure pursuit is forward-only: if the target is well off the current
  // heading (behind, or sharply to one side) the arc it would follow is
  // tiny and the robot crawls off in the wrong direction. A differential
  // -drive base can spin in place, so above turnInPlaceRad just rotate
  // toward the target first, then hand back to pursuit once roughly aligned.
  const bearing = Math.atan2(local.y, local.x);
  if (Math.abs(bearing) > turnInPlaceRad) {
    const s = Math.sign(bearing); // +1: target to the left -> turn left (omega>0 -> vR>vL)
    return { left: -s * turnGain, right: s * turnGain, atGoal: false, lookaheadIndex: index };
  }

  const l2 = local.x * local.x + local.y * local.y;
  // Standard pure-pursuit curvature: kappa = 2y / L^2 (Coulter 1992). l2 can't
  // be 0 here -- findLookaheadPoint only returns a point already >= some
  // positive distance away, or the still-distant goal (guarded above).
  const curvature = (2 * local.y) / l2;

  const omega = v * curvature;
  const halfTrack = geometry.wheelSeparation / 2;
  const wheelLinearL = v - omega * halfTrack;
  const wheelLinearR = v + omega * halfTrack;

  const maxWheelLinear = ((geometry.maxWheelRpm * 2 * Math.PI) / 60) * geometry.wheelRadius;
  const clamp = (x) => Math.max(-1, Math.min(1, x));

  return {
    left: clamp(wheelLinearL / maxWheelLinear),
    right: clamp(wheelLinearR / maxWheelLinear),
    atGoal: false,
    lookaheadIndex: index,
  };
}

// --- bus wiring ---------------------------------------------------------

/** Distance from a pose to the closest point on the polyline (m). */
export function distanceToPath(path, pose) {
  if (!path || path.length === 0) return Infinity;
  let best = Infinity;
  for (let i = 0; i < path.length; i++) {
    const [ax, ay] = path[i];
    if (i + 1 < path.length) {
      const [bx, by] = path[i + 1];
      const dx = bx - ax, dy = by - ay;
      const l2 = dx * dx + dy * dy;
      const t = l2 > 0 ? Math.max(0, Math.min(1, ((pose.x - ax) * dx + (pose.y - ay) * dy) / l2)) : 0;
      best = Math.min(best, Math.hypot(pose.x - (ax + t * dx), pose.y - (ay + t * dy)));
    } else {
      best = Math.min(best, Math.hypot(pose.x - ax, pose.y - ay));
    }
  }
  return best;
}

export class PathFollowerNode {
  /**
   * maxDeviationM: safety stop -- if the (fused) pose drifts further than this
   * from the given path the node drops the path, publishes zero velocity and
   * calls onAbort({ reason: 'pathDeviation', distance }). A robot pushed off
   * its path (collision, slipping wheels, a bad localization jump) must not
   * keep driving toward a lookahead point through whatever is in the way.
   * 0/undefined disables the check.
   */
  constructor(bus, { pathTopic, poseTopic, cmdTopic, geometry, onGoalReached, onAbort, maxDeviationM = 0, ...pursuitOptions } = {}) {
    if (!pathTopic || !poseTopic || !cmdTopic || !geometry) {
      throw new Error('PathFollowerNode requires pathTopic, poseTopic, cmdTopic, geometry');
    }
    this._bus = bus;
    this._cmdTopic = cmdTopic;
    this._geometry = geometry;
    this._pursuitOptions = pursuitOptions;
    this._onGoalReached = onGoalReached;
    this._onAbort = onAbort;
    this._maxDeviationM = maxDeviationM;
    this._path = null;
    this._lookaheadIndex = 0;

    this._unsubPath = bus.subscribe(pathTopic, (msg) => this._onPath(msg));
    this._unsubPose = bus.subscribe(poseTopic, (pose) => this._onPose(pose));
  }

  _onPath({ path }) {
    this._path = Array.isArray(path) && path.length > 0 ? path : null;
    this._lookaheadIndex = 0;
  }

  _onPose(pose) {
    if (!this._path) return;
    if (this._maxDeviationM > 0) {
      const dev = distanceToPath(this._path, pose);
      if (dev > this._maxDeviationM) {
        this._path = null;
        this._bus.publish(this._cmdTopic, { left: 0, right: 0 });
        if (this._onAbort) this._onAbort({ reason: 'pathDeviation', distance: dev, limit: this._maxDeviationM });
        return;
      }
    }
    const result = pursuitStep(this._path, pose, this._geometry, {
      ...this._pursuitOptions,
      startIndex: this._lookaheadIndex,
    });
    this._lookaheadIndex = result.lookaheadIndex;
    this._bus.publish(this._cmdTopic, { left: result.left, right: result.right });
    if (result.atGoal) {
      this._path = null;
      if (this._onGoalReached) this._onGoalReached();
    }
  }

  stop() {
    this._unsubPath();
    this._unsubPose();
    this._bus.publish(this._cmdTopic, { left: 0, right: 0 });
  }
}
