// OdometryNode — roadmap.md Phase 7's "OdometryNode: 엔코더 → odom pose".
//
// Polls the Roboteq controller's encoder counts (`?C`, already answered by
// both robot-base/sim and ros-chromium/simulator, but never consumed by any
// JS client until now — see device-abstraction/drive-device.js's TODO
// comment on readback) and dead-reckons a pose from the count deltas using
// standard differential-drive odometry. This pose is *relative to wherever
// OdometryNode started counting* and drifts over time/distance — it is not
// a substitute for an absolute fix, which is exactly why PoseFusionNode
// exists (see pose-fusion-node.js) to combine this with periodic corrections.

import { encodeCommand } from '../../transport/src/roboteq.js';

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
  constructor(bus, transport, { poseTopic, geometry, pollHz = 20 } = {}) {
    if (!poseTopic || !geometry) {
      throw new Error('OdometryNode requires poseTopic and geometry');
    }
    this._bus = bus;
    this._poseTopic = poseTopic;
    this._geometry = geometry;
    this._pose = { x: 0, y: 0, theta: 0 };
    this._lastCounts = null;

    // Roboteq ASCII has no request/response correlation (no message ids) --
    // every "?C" reply just shows up as the next "C=..." line, same pattern
    // roboteq-smoke.mjs already relies on for "?FID" during the handshake.
    this._onMessage = (msg) => {
      if (msg.type !== 'reply' || msg.key !== 'C' || msg.values.length < 2) return;
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
      transport.send(encodeCommand('?C'));
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
