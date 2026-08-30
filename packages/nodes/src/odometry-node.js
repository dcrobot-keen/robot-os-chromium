// OdometryNode — roadmap.md Phase 7's "OdometryNode: 엔코더 → odom pose".
//
// Polls the motor controller's encoder counts and dead-reckons a pose from
// the count deltas using standard differential-drive odometry. This pose is
// *relative to wherever OdometryNode started counting* and drifts over
// time/distance — it is not a substitute for an absolute fix, which is
// exactly why PoseFusionNode exists (see pose-fusion-node.js) to combine
// this with periodic corrections.
//
// Protocol-agnostic: the query string comes from the manifest
// (`drive.readback.encoder` — "?C" for Roboteq) and is turned into wire
// bytes by `transport.encode()`, so this node has no codec import. The
// decoder is expected to surface counts as `{type:'reply', key:'C',
// values:[left,right]}` regardless of wire protocol (see codecs.js).

// Wheel rotation (radians, from encoder counts) -> linear distance travelled
// by that wheel (meters). Pure function so the arithmetic is unit-testable
// without a transport or a bus.
export function countsToWheelDistances(deltaCounts, geometry) {
  const metersPerCount = (2 * Math.PI * geometry.wheelRadius) / geometry.countsPerRev;
  return {
    left: deltaCounts.left * metersPerCount,
    right: deltaCounts.right * metersPerCount,
  };
}

// One differential-drive odometry step: given the previous pose and how far
// each wheel just travelled, return the new pose. Uses the midpoint heading
// for the translation step (standard "improved Euler" odometry integration)
// rather than the heading before or after the turn, which is measurably
// more accurate for the same encoder-poll rate without needing a smaller
// step size.
export function integrateOdometry(pose, wheelDistances, geometry) {
  const dCenter = (wheelDistances.left + wheelDistances.right) / 2;
  const dTheta = (wheelDistances.right - wheelDistances.left) / geometry.wheelSeparation;
  const midTheta = pose.theta + dTheta / 2;
  return {
    x: pose.x + dCenter * Math.cos(midTheta),
    y: pose.y + dCenter * Math.sin(midTheta),
    theta: pose.theta + dTheta,
  };
}

export class OdometryNode {
  constructor(
    bus,
    transport,
    { poseTopic, geometry, pollHz = 20, encoderQuery = '?C', encoderReplyKey = 'C' } = {},
  ) {
    if (!poseTopic || !geometry) {
      throw new Error('OdometryNode requires poseTopic and geometry');
    }
    this._bus = bus;
    this._poseTopic = poseTopic;
    this._geometry = geometry;
    this._encoderQuery = encoderQuery;
    this._pose = { x: 0, y: 0, theta: 0 };
    this._lastCounts = null;

    // Roboteq ASCII has no request/response correlation (no message ids) --
    // every "?C" reply just shows up as the next "C=..." line, same pattern
    // roboteq-smoke.mjs already relies on for "?FID" during the handshake.
    // A different codec surfaces its encoder read under whatever key it
    // chooses; pass encoderReplyKey to match it.
    this._onMessage = (msg) => {
      if (msg.type !== 'reply' || msg.key !== encoderReplyKey || msg.values.length < 2) return;
      const counts = { left: msg.values[0], right: msg.values[1] };
      if (this._lastCounts) {
        const delta = {
          left: counts.left - this._lastCounts.left,
          right: counts.right - this._lastCounts.right,
        };
        const wheelDistances = countsToWheelDistances(delta, geometry);
        this._pose = integrateOdometry(this._pose, wheelDistances, geometry);
        this._bus.publish(poseTopic, { ...this._pose });
      }
      this._lastCounts = counts;
    };
    transport.onMessage(this._onMessage);

    this._transport = transport;
    this._interval = setInterval(() => {
      transport.send(transport.encode(this._encoderQuery));
    }, 1000 / pollHz);
  }

  // Re-anchors the odometry frame (e.g. to a known start pose) without
  // touching the encoder baseline -- the next "?C" reply still diffs
  // against whatever counts were last seen, only the integrated pose jumps.
  resetPose(pose) {
    this._pose = { ...pose };
  }

  getPose() {
    return { ...this._pose };
  }

  stop() {
    clearInterval(this._interval);
  }
}
