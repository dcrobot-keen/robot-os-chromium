// PoseFusionNode — this stack's replacement for robot_localization's EKF
// (vps-system/ros2_ws's dc_vps_bridge feeds a `vps_pose` into exactly that
// node; see architecture-improvements.md for the fuller comparison).
//
// It fuses OdometryNode's continuous-but-drifting pose with periodic
// absolute corrections (VPS `/localize`, or in the simulator a throttled +
// noisy stand-in for it) using a Kalman filter. Deliberately a *decoupled*
// filter -- x, y, and theta each get their own independent scalar Kalman
// update rather than one dense 3x3 covariance matrix. A full joint EKF
// earns its complexity when a nonlinear measurement model (e.g. fusing a
// raw IMU) forces you to linearize; here both inputs (odometry pose deltas,
// absolute pose fixes) are already in the same x/y/theta space, so three
// independent 1-D filters give the same qualitative behavior -- uncertainty
// grows with distance travelled, shrinks and pulls the estimate toward each
// new fix, weighted by relative confidence -- for much less code and no
// matrix math to get wrong. GET_IMU readback doesn't exist yet either (see
// robot-os-chromium/plan.md), so there's no third sensor that would justify
// the extra machinery today.

function wrapAngle(a) {
  return Math.atan2(Math.sin(a), Math.cos(a));
}

// Predict step: apply an odometry pose delta (already computed by the
// caller from two consecutive OdometryNode readings) and grow uncertainty.
// Growth is proportional to how far the robot actually moved, not to wall
//-clock time -- a robot sitting still doesn't lose confidence in its
// position, but driving 10m on bad wheels does. `processNoise` is
// { perMeter, perRadian, floor } (floor keeps variance from ever hitting
// exactly 0, which would make correct() divide by zero).
export function predict(state, odomDelta, processNoise) {
  const dist = Math.hypot(odomDelta.dx, odomDelta.dy);
  const rot = Math.abs(odomDelta.dtheta);
  return {
    x: state.x + odomDelta.dx,
    y: state.y + odomDelta.dy,
    theta: wrapAngle(state.theta + odomDelta.dtheta),
    varX: state.varX + processNoise.perMeter * dist + processNoise.floor,
    varY: state.varY + processNoise.perMeter * dist + processNoise.floor,
    varTheta: state.varTheta + processNoise.perRadian * rot + processNoise.floor,
  };
}

// One scalar Kalman update: blend prior estimate `x`(variance `varX`) with
// measurement `z` (variance `varZ`). Returns the posterior value + variance.
function kalman1d(x, varX, z, varZ) {
  const k = varX / (varX + varZ); // Kalman gain: how much we trust the new fix
  return { value: x + k * (z - x), variance: (1 - k) * varX };
}

// Correct step: blend in an absolute pose fix (VPS or equivalent).
// `measurementNoise` is { x, y, theta } variances for that fix.
export function correct(state, measurement, measurementNoise) {
  const kx = kalman1d(state.x, state.varX, measurement.x, measurementNoise.x);
  const ky = kalman1d(state.y, state.varY, measurement.y, measurementNoise.y);
  // theta wraps at +/-pi, so the "measurement" fed to the scalar filter is
  // the shortest-path angular innovation, not the raw heading difference.
  const innovation = wrapAngle(measurement.theta - state.theta);
  const kTheta = kalman1d(0, state.varTheta, innovation, measurementNoise.theta);
  return {
    x: kx.value,
    y: ky.value,
    theta: wrapAngle(state.theta + kTheta.value),
    varX: kx.variance,
    varY: ky.variance,
    varTheta: kTheta.variance,
  };
}

const DEFAULT_PROCESS_NOISE = { perMeter: 0.02, perRadian: 0.01, floor: 1e-6 };
const DEFAULT_MEASUREMENT_NOISE = { x: 0.01, y: 0.01, theta: 0.001 };
// Callers who actually know their start pose (docked at a known charging
// station, say) can pass a small initialVariance. The class can't safely
// default to that, though -- a caller with no idea where it started (this
// stack's real use so far: sim-driver, which only knows the sim's ground
// truth by cheating, and a real robot never gets to) that leaves the
// hardcoded-0 default in place gets a filter that trusts its (possibly
// wrong) initialPose completely, so the Kalman gain on the very first
// correction stays ~0 and the wrong guess never gets fixed. Defaulting to
// "not very sure yet" makes that first correction do its job either way.
const DEFAULT_INITIAL_VARIANCE = { x: 4, y: 4, theta: (Math.PI / 2) ** 2 };

export class PoseFusionNode {
  constructor(
    bus,
    {
      odomTopic,
      correctionTopic,
      fusedTopic,
      initialPose = { x: 0, y: 0, theta: 0 },
      initialVariance = DEFAULT_INITIAL_VARIANCE,
      processNoise = DEFAULT_PROCESS_NOISE,
      measurementNoise = DEFAULT_MEASUREMENT_NOISE,
    } = {}
  ) {
    if (!odomTopic || !correctionTopic || !fusedTopic) {
      throw new Error('PoseFusionNode requires odomTopic, correctionTopic, fusedTopic');
    }
    this._bus = bus;
    this._fusedTopic = fusedTopic;
    this._processNoise = processNoise;
    this._measurementNoise = measurementNoise;
    this._state = {
      ...initialPose,
      varX: initialVariance.x,
      varY: initialVariance.y,
      varTheta: initialVariance.theta,
    };
    this._lastOdom = null;

    this._unsubOdom = bus.subscribe(odomTopic, (odomPose) => {
      if (this._lastOdom) {
        const delta = {
          dx: odomPose.x - this._lastOdom.x,
          dy: odomPose.y - this._lastOdom.y,
          dtheta: wrapAngle(odomPose.theta - this._lastOdom.theta),
        };
        this._state = predict(this._state, delta, this._processNoise);
        this._publish();
      }
      this._lastOdom = odomPose;
    });

    this._unsubCorrection = bus.subscribe(correctionTopic, (measurement) => {
      this._state = correct(this._state, measurement, this._measurementNoise);
      this._publish();
    });
  }

  _publish() {
    const { x, y, theta, varX, varY, varTheta } = this._state;
    this._bus.publish(this._fusedTopic, { x, y, theta, varX, varY, varTheta });
  }

  getState() {
    return { ...this._state };
  }

  stop() {
    this._unsubOdom();
    this._unsubCorrection();
  }
}
