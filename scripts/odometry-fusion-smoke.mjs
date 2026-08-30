// Smoke test for OdometryNode's dead-reckoning and PoseFusionNode's
// Kalman-style correction (this repo's replacement for robot_localization's
// EKF) -- no simulator, no browser, no real transport.
//
//   node scripts/odometry-fusion-smoke.mjs
import {
  countsToWheelDistances,
  integrateOdometry,
  OdometryNode,
  predict,
  correct,
  PoseFusionNode,
} from '@ros-chromium/nodes';
import { LocalBus } from '@ros-chromium/bus';

let failures = 0;
function check(name, cond, extra = '') {
  console.log(`${cond ? 'PASS' : 'FAIL'}  ${name}${extra ? '  (' + extra + ')' : ''}`);
  if (!cond) failures++;
}

const TB3_GEOMETRY = { wheelRadius: 0.033, wheelSeparation: 0.16, countsPerRev: 4096 };

// --- 1. countsToWheelDistances: one full revolution -> wheel circumference ---
{
  const dist = countsToWheelDistances({ left: 4096, right: 0 }, TB3_GEOMETRY);
  const expected = 2 * Math.PI * TB3_GEOMETRY.wheelRadius;
  check(
    'countsToWheelDistances: one full revolution = wheel circumference',
    Math.abs(dist.left - expected) < 1e-9,
    `got ${dist.left.toFixed(4)}, expected ${expected.toFixed(4)}`
  );
  check('countsToWheelDistances: zero counts on the other wheel', dist.right === 0);
}

// --- 2. integrateOdometry: equal wheel distances drives straight ahead ---
{
  const pose = integrateOdometry({ x: 0, y: 0, theta: 0 }, { left: 1, right: 1 }, TB3_GEOMETRY);
  check('integrateOdometry: equal wheels -> y unchanged', Math.abs(pose.y) < 1e-9, `got y=${pose.y}`);
  check('integrateOdometry: equal wheels -> heading unchanged', Math.abs(pose.theta) < 1e-9);
  check('integrateOdometry: equal wheels -> forward distance is dCenter', Math.abs(pose.x - 1) < 1e-9);
}

// --- 3. integrateOdometry: right wheel faster -> theta increases (turns left),
// matching PathFollowerNode's omega=(vR-vL)/L convention ---
{
  const pose = integrateOdometry({ x: 0, y: 0, theta: 0 }, { left: 0.5, right: 1 }, TB3_GEOMETRY);
  check('integrateOdometry: right wheel faster -> theta increases', pose.theta > 0, `got theta=${pose.theta.toFixed(4)}`);
}

// --- 4. OdometryNode over a fake transport: encoder count deltas -> bus pose ---
{
  const bus = new LocalBus('odom-smoke');
  const poses = [];
  bus.subscribe('odom', (p) => poses.push(p));

  let onMessageCb = null;
  const fakeTransport = {
    onMessage(cb) {
      onMessageCb = cb;
    },
    send() {},
    encode: (spec) => spec, // identity — this test never inspects the wire bytes
  };

  const node = new OdometryNode(bus, fakeTransport, { poseTopic: 'odom', geometry: TB3_GEOMETRY, pollHz: 1000 });
  // the first reading only establishes the baseline -- there's no prior
  // count to diff against yet, so nothing should publish.
  onMessageCb({ type: 'reply', key: 'C', values: [1000, 1000] });
  check('OdometryNode: first encoder reading only sets the baseline (no pose yet)', poses.length === 0);

  // both wheels advance by one full revolution -> straight ahead by the wheel circumference
  onMessageCb({ type: 'reply', key: 'C', values: [1000 + 4096, 1000 + 4096] });
  check('OdometryNode: publishes a pose once a second reading arrives', poses.length === 1);
  const expectedDist = 2 * Math.PI * TB3_GEOMETRY.wheelRadius;
  check(
    'OdometryNode: one full wheel revolution on both sides -> x advances by the wheel circumference',
    Math.abs(poses[0].x - expectedDist) < 1e-9,
    `got x=${poses[0].x.toFixed(4)}`
  );

  node.stop();
  bus.close();
}

// --- 5. predict(): uncertainty grows with distance travelled, not with time ---
{
  const state = { x: 0, y: 0, theta: 0, varX: 0, varY: 0, varTheta: 0 };
  const processNoise = { perMeter: 0.02, perRadian: 0.01, floor: 1e-6 };
  const moved = predict(state, { dx: 1, dy: 0, dtheta: 0 }, processNoise);
  const stayed = predict(state, { dx: 0, dy: 0, dtheta: 0 }, processNoise);
  check(
    'predict: driving 1m increases varX by ~perMeter',
    Math.abs(moved.varX - (processNoise.perMeter + processNoise.floor)) < 1e-9,
    `got ${moved.varX}`
  );
  check('predict: standing still barely grows variance (floor only)', stayed.varX === processNoise.floor);
  check('predict: position advances by the odom delta', Math.abs(moved.x - 1) < 1e-9);
}

// --- 6. correct(): Kalman gain blends toward whichever source is more confident ---
{
  const measurementNoise = { x: 1, y: 1, theta: 1 };
  const equal = correct({ x: 0, y: 0, theta: 0, varX: 1, varY: 1, varTheta: 1 }, { x: 10, y: 0, theta: 0 }, measurementNoise);
  check('correct: equal variances -> posterior is the average of prior and measurement', Math.abs(equal.x - 5) < 1e-9, `got x=${equal.x}`);
  check('correct: equal variances -> posterior variance halves', Math.abs(equal.varX - 0.5) < 1e-9);

  const driftedOdom = correct(
    { x: 0, y: 0, theta: 0, varX: 100, varY: 100, varTheta: 100 },
    { x: 10, y: 0, theta: 0 },
    measurementNoise
  );
  check('correct: a much-more-confident measurement pulls the estimate close to it', driftedOdom.x > 9, `got x=${driftedOdom.x.toFixed(3)}`);

  const confidentOdom = correct(
    { x: 0, y: 0, theta: 0, varX: 0.001, varY: 0.001, varTheta: 0.001 },
    { x: 10, y: 0, theta: 0 },
    measurementNoise
  );
  check('correct: a confident prior barely moves toward a noisy measurement', confidentOdom.x < 0.1, `got x=${confidentOdom.x.toFixed(4)}`);
}

// --- 7. correct(): theta innovation wraps across the +/-pi seam ---
{
  const nearPi = correct(
    { x: 0, y: 0, theta: Math.PI - 0.05, varX: 1, varY: 1, varTheta: 1 },
    { x: 0, y: 0, theta: -Math.PI + 0.05 }, // 0.1 rad away the "short way", across the seam
    { x: 1, y: 1, theta: 1 }
  );
  const distanceFromSeam = Math.min(Math.abs(nearPi.theta - Math.PI), Math.abs(nearPi.theta + Math.PI));
  check(
    'correct: theta wraps across +/-pi instead of averaging the long way around',
    distanceFromSeam < 0.1,
    `got theta=${nearPi.theta.toFixed(4)}`
  );
}

// --- 8. end-to-end: does fusion actually reduce drift vs. raw odometry alone? ---
// This is the point of the whole exercise -- not "does the math run" but
// "does periodically correcting with an absolute fix keep the estimate
// closer to the truth than dead-reckoning alone", the same job
// robot_localization's EKF does with vps_pose in dc_vps_bridge.
{
  const bus = new LocalBus('fusion-smoke');
  const fusedPoses = [];
  bus.subscribe('fused', (p) => fusedPoses.push(p));

  const fusion = new PoseFusionNode(bus, {
    odomTopic: 'odom',
    correctionTopic: 'correction',
    fusedTopic: 'fused',
    processNoise: { perMeter: 0.02, perRadian: 0.01, floor: 1e-6 },
    measurementNoise: { x: 0.0025, y: 0.0025, theta: 0.0009 }, // ~5cm / ~1.7deg sigma
  });

  const dt = 0.1;
  const v = 0.1; // m/s, straight line along +x
  const steps = 200; // 20s
  const correctionEveryNSteps = 20; // ~2s, matching vps-capture.html's default cadence
  const slipFactor = 1.03; // wheels over-report distance by 3%, same order as
  // the simulator's "default" noise preset (odom.slipSigma) -- see
  // ros-chromium/simulator/src/noise.js.

  let truePose = { x: 0, y: 0, theta: 0 };
  let odomPose = { x: 0, y: 0, theta: 0 }; // what OdometryNode reports: biased, never corrected

  for (let i = 1; i <= steps; i++) {
    truePose = { x: truePose.x + v * dt, y: 0, theta: 0 };
    odomPose = { x: odomPose.x + v * dt * slipFactor, y: 0, theta: 0 };
    bus.publish('odom', { ...odomPose });

    if (i % correctionEveryNSteps === 0) {
      // a small, deterministic (not random, so this test can't flake)
      // measurement offset stands in for VPS's own error.
      bus.publish('correction', { x: truePose.x + 0.005, y: 0.002, theta: 0.001 });
    }
  }

  const rawOdomError = Math.abs(odomPose.x - truePose.x);
  const fusedError = Math.abs(fusedPoses.at(-1).x - truePose.x);

  check(
    'end-to-end: fused estimate ends up much closer to true pose than uncorrected odometry',
    fusedError < rawOdomError * 0.3,
    `raw odom error=${rawOdomError.toFixed(4)}m, fused error=${fusedError.toFixed(4)}m`
  );
  check('end-to-end: fusion still tracks forward motion (not stuck at origin)', fusedPoses.at(-1).x > truePose.x * 0.8);

  fusion.stop();
  bus.close();
}

// --- 9. PoseFusionNode: a cold start with no clue where it actually is
// still converges on the first correction (regression guard for the
// zero-initial-variance bug this class used to have -- see the comment on
// DEFAULT_INITIAL_VARIANCE in pose-fusion-node.js) ---
{
  const bus = new LocalBus('cold-start-smoke');
  const fusedPoses = [];
  bus.subscribe('fused', (p) => fusedPoses.push(p));

  const fusion = new PoseFusionNode(bus, {
    odomTopic: 'odom',
    correctionTopic: 'correction',
    fusedTopic: 'fused',
    // assumes it started at the origin, but it actually started 5m away and
    // hasn't moved yet -- no odometry reading has arrived to grow the
    // filter's uncertainty, so this has to rely entirely on
    // DEFAULT_INITIAL_VARIANCE already being "not very sure" out of the box.
    initialPose: { x: 0, y: 0, theta: 0 },
  });
  bus.publish('correction', { x: 5, y: 5, theta: 0 });
  check(
    'PoseFusionNode: cold start converges toward the first correction instead of staying near a wrong initial guess',
    fusedPoses.length === 1 && fusedPoses[0].x > 4,
    `got x=${fusedPoses[0]?.x}`
  );
  fusion.stop();
  bus.close();
}

console.log(failures === 0 ? '\nall odometry/fusion smoke checks passed' : `\n${failures} check(s) failed`);
process.exit(failures === 0 ? 0 : 1);
