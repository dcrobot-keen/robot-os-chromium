// Smoke test for PathFollowerNode's pure-pursuit control loop -- no
// simulator, no browser. Runs a tiny standalone kinematic integrator (the
// same differential-drive equations simulator/src/robot.js uses, but
// reimplemented here in ~10 lines rather than importing across the
// filesystem boundary the two repos deliberately don't share) and checks
// that the *actual, integrated* trajectory converges on the goal -- not
// just that pursuitStep() returns plausible-looking numbers for one pose.
//
//   node scripts/path-follower-smoke.mjs
import { pursuitStep, toRobotFrame, findLookaheadPoint, PathFollowerNode } from '@ros-chromium/nodes';
import { LocalBus } from '@ros-chromium/bus';

let failures = 0;
function check(name, cond, extra = '') {
  console.log(`${cond ? 'PASS' : 'FAIL'}  ${name}${extra ? '  (' + extra + ')' : ''}`);
  if (!cond) failures++;
}

const TB3_GEOMETRY = { wheelRadius: 0.033, wheelSeparation: 0.16, maxWheelRpm: 63.7 };
const maxWheelLinear = ((TB3_GEOMETRY.maxWheelRpm * 2 * Math.PI) / 60) * TB3_GEOMETRY.wheelRadius;

// One differential-drive integration step (no walls/collision -- this test
// is about convergence of the control law, not the physics engine).
function integrate(pose, left, right, dt) {
  const vL = left * maxWheelLinear;
  const vR = right * maxWheelLinear;
  const v = (vL + vR) / 2;
  const omega = (vR - vL) / TB3_GEOMETRY.wheelSeparation;
  const theta = pose.theta + omega * dt;
  return { x: pose.x + v * Math.cos(theta) * dt, y: pose.y + v * Math.sin(theta) * dt, theta };
}

// --- 1. toRobotFrame: a point straight ahead has local y = 0 ---
{
  const pose = { x: 1, y: 1, theta: 0 };
  const local = toRobotFrame([3, 1], pose);
  check('toRobotFrame: point straight ahead has y≈0', Math.abs(local.y) < 1e-9, `got y=${local.y}`);
  check('toRobotFrame: forward distance is correct', Math.abs(local.x - 2) < 1e-9, `got x=${local.x}`);
}
{
  // facing +y (theta=pi/2), a point to the world +x is to the robot's right (local y < 0)
  const pose = { x: 0, y: 0, theta: Math.PI / 2 };
  const local = toRobotFrame([1, 0], pose);
  check('toRobotFrame: world +x while facing +y is to the robot\'s right', local.y < 0, `got y=${local.y}`);
}

// --- 2. findLookaheadPoint: falls back to the final point near the path's end ---
{
  const path = [[0, 0], [1, 0], [2, 0]];
  // fromIndex=2: already tracking the last segment, and the remaining
  // distance to path[2] (0.05m) is under the 0.3m lookahead -- nothing
  // ahead qualifies, so it must fall back to the final point.
  const { point, index } = findLookaheadPoint(path, { x: 1.95, y: 0, theta: 0 }, 0.3, 2);
  check('findLookaheadPoint: falls back to final point when nothing is far enough ahead', point === path[2] && index === 2);
}

// --- 3. pursuitStep: straight-ahead path drives straight (left ≈ right) ---
{
  const path = [[0, 0], [1, 0], [2, 0], [3, 0]];
  const result = pursuitStep(path, { x: 0, y: 0, theta: 0 }, TB3_GEOMETRY);
  check('pursuitStep: dead-ahead path -> left ≈ right (no turn)', Math.abs(result.left - result.right) < 1e-6, `left=${result.left.toFixed(4)} right=${result.right.toFixed(4)}`);
  check('pursuitStep: moving forward (both positive)', result.left > 0 && result.right > 0);
}

// --- 4. pursuitStep: goal reached within tolerance stops the robot ---
{
  const path = [[0, 0], [1, 0]];
  const result = pursuitStep(path, { x: 0.97, y: 0, theta: 0 }, TB3_GEOMETRY, { goalToleranceM: 0.1 });
  check('pursuitStep: within goal tolerance -> atGoal + zero command', result.atGoal && result.left === 0 && result.right === 0);
}

// --- 5. pursuitStep: robot to the left of the path turns right (curvature sign) ---
{
  // path runs along +x; robot is offset to +y (its left, since it faces +x) of it,
  // so it needs to turn right (clockwise) to get back on the path. Standard
  // differential-drive kinematics (omega = (vR - vL) / L) means a clockwise
  // turn (omega < 0) needs vR < vL -- the LEFT wheel is the faster one, not
  // the right (the "outer" wheel of a turn is the faster one, and the left
  // side is outer when turning right).
  const path = [[0, 0], [1, 0], [2, 0]];
  const result = pursuitStep(path, { x: 0, y: 0.3, theta: 0 }, TB3_GEOMETRY, { lookaheadM: 0.5 });
  check('pursuitStep: offset to the left steers right (left wheel is the outer/faster one)', result.left > result.right, `left=${result.left.toFixed(4)} right=${result.right.toFixed(4)}`);
}

// --- 6. end-to-end convergence: integrate real control loop ticks, must reach the goal ---
{
  const path = [[0, 0], [1, 0], [1, 1], [0, 1]]; // an L-shaped path
  let pose = { x: 0, y: 0, theta: 0 };
  const dt = 0.05; // 20 Hz, matches the simulator's scanHz
  const geometry = TB3_GEOMETRY;
  let startIndex = 0;
  let reached = false;
  let ticks = 0;
  const maxTicks = 2000; // 100s of sim time -- generous, cruise is 0.12 m/s over a ~3m path
  while (ticks < maxTicks) {
    const result = pursuitStep(path, pose, geometry, { startIndex, goalToleranceM: 0.08 });
    startIndex = result.lookaheadIndex;
    if (result.atGoal) {
      reached = true;
      break;
    }
    pose = integrate(pose, result.left, result.right, dt);
    ticks++;
  }
  const goal = path[path.length - 1];
  const finalDist = Math.hypot(goal[0] - pose.x, goal[1] - pose.y);
  check(
    'end-to-end: integrating the control loop actually reaches the goal',
    reached,
    `${ticks} ticks, final dist to goal ${finalDist.toFixed(3)}m, final pose (${pose.x.toFixed(2)}, ${pose.y.toFixed(2)})`
  );
}

// --- 7. PathFollowerNode over a real LocalBus: path + pose in -> cmd_vel out ---
{
  const bus = new LocalBus('path-follower-smoke');
  const commands = [];
  bus.subscribe('cmd_vel', (c) => commands.push(c));
  const node = new PathFollowerNode(bus, {
    pathTopic: 'path',
    poseTopic: 'pose',
    cmdTopic: 'cmd_vel',
    geometry: TB3_GEOMETRY,
  });

  bus.publish('path', { path: [[0, 0], [1, 0]] });
  bus.publish('pose', { x: 0, y: 0, theta: 0 });
  check('PathFollowerNode: publishes a non-zero command for a fresh path', commands.length === 1 && (commands[0].left !== 0 || commands[0].right !== 0));

  bus.publish('pose', { x: 0.97, y: 0, theta: 0 }); // inside default goal tolerance
  check('PathFollowerNode: zero command once the goal is reached', commands.at(-1).left === 0 && commands.at(-1).right === 0);

  node.stop();
  bus.close();
}

console.log(failures === 0 ? '\nall path-follower smoke checks passed' : `\n${failures} check(s) failed`);
process.exit(failures === 0 ? 0 : 1);
