// Smoke test for LocalizationNode (roadmap.md Phase 9, step 5) -- no
// browser, no simulator. A box room, a "true" trajectory, and an odometry
// stream that drifts away from it; LocalizationNode scan-matches against
// the box's likelihood field and its corrections should keep the estimate
// locked to the true pose while raw odom wanders off. Also checks odom
// carry-forward between scans, setPose(), and lost/relocalize.
//
//   node scripts/localization-node-smoke.mjs
import {
  gridConfigFromBounds,
  cellIndex,
  buildLikelihoodField,
  LocalizationNode,
} from '@ros-chromium/nodes';
import { LocalBus } from '@ros-chromium/bus';

let failures = 0;
const check = (name, cond, extra = '') => {
  console.log(`${cond ? 'PASS' : 'FAIL'}  ${name}${extra ? '  (' + extra + ')' : ''}`);
  if (!cond) failures++;
};
const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));

// --- box room + raycaster --------------------------------------------
const WALLS = [
  [0, 0, 7, 0], [7, 0, 7, 5], [7, 5, 0, 5], [0, 5, 0, 0],
  [4, 0, 4, 2.5], [4, 2.5, 5.5, 2.5], // an L to give yaw + x/y signal
];
function raycast(ox, oy, ang, max) {
  const dx = Math.cos(ang), dy = Math.sin(ang);
  let best = max;
  for (const [x1, y1, x2, y2] of WALLS) {
    const sx = x2 - x1, sy = y2 - y1;
    const denom = dx * sy - dy * sx;
    if (Math.abs(denom) < 1e-12) continue;
    const t = ((x1 - ox) * sy - (y1 - oy) * sx) / denom;
    const u = ((x1 - ox) * dy - (y1 - oy) * dx) / denom;
    if (t >= 0 && t < best && u >= 0 && u <= 1) best = t;
  }
  return best;
}
function scanAt(pose, count = 240, rangeMax = 4.0) {
  const inc = (2 * Math.PI) / count;
  const ranges = new Array(count);
  for (let i = 0; i < count; i++) {
    ranges[i] = Math.max(0.12, Math.min(rangeMax, raycast(pose.x, pose.y, pose.theta + i * inc, rangeMax)));
  }
  return { angleMin: 0, angleIncrement: inc, rangeMin: 0.12, rangeMax, ranges };
}

// --- likelihood field from the box ----------------------------------
const grid = gridConfigFromBounds({ minX: -0.4, minY: -0.4, maxX: 7.4, maxY: 5.4 }, 0.05);
const occ = new Uint8Array(grid.cols * grid.rows);
for (let row = 0; row < grid.rows; row++) {
  for (let col = 0; col < grid.cols; col++) {
    const px = grid.originX + (col + 0.5) * grid.cellSize;
    const py = grid.originY + (row + 0.5) * grid.cellSize;
    for (const [x1, y1, x2, y2] of WALLS) {
      const vx = x2 - x1, vy = y2 - y1;
      const len2 = vx * vx + vy * vy || 1e-9;
      let t = ((px - x1) * vx + (py - y1) * vy) / len2;
      t = Math.max(0, Math.min(1, t));
      if ((px - (x1 + t * vx)) ** 2 + (py - (y1 + t * vy)) ** 2 <= (grid.cellSize * 0.8) ** 2) {
        occ[cellIndex(grid, col, row)] = 1; break;
      }
    }
  }
}
const lf = buildLikelihoodField(grid, occ, { sigmaM: 0.08 });

// --- run: true trajectory + drifting odom -------------------------------
const bus = new LocalBus('localization-smoke');
const corrections = [];
const events = [];
bus.subscribe('correction', (c) => corrections.push(c));

const START = { x: 2.0, y: 2.0, theta: 0.0 };
const node = new LocalizationNode(bus, {
  scanTopic: 'scan', odomTopic: 'odom', correctionTopic: 'correction',
  likelihoodField: lf, initialPose: { ...START },
  minScore: 0.4, lostAfter: 3,
  onEvent: (e) => events.push(e),
});

let truth = { ...START };
let odom = { ...START };       // starts aligned, then drifts
const SLIP = 1.06;            // odom over-reports forward by 6%
const YAW_BIAS = 0.015;       // and adds 0.015 rad/step of phantom yaw

function step(dForward, dTheta) {
  // advance truth
  truth = { x: truth.x + dForward * Math.cos(truth.theta), y: truth.y + dForward * Math.sin(truth.theta), theta: wrap(truth.theta + dTheta) };
  // advance odom with slip + yaw bias
  const of = dForward * SLIP;
  odom = { x: odom.x + of * Math.cos(odom.theta), y: odom.y + of * Math.sin(odom.theta), theta: wrap(odom.theta + dTheta + YAW_BIAS) };
  bus.publish('odom', { ...odom });          // OdometryNode would publish this
  bus.publish('scan', scanAt(truth));        // real scan is from the true pose
}

// drive a little loop
for (let i = 0; i < 12; i++) step(0.15, 0.0);
for (let i = 0; i < 8; i++) step(0.12, 0.12);
for (let i = 0; i < 12; i++) step(0.15, 0.0);

const last = corrections.at(-1);
const corrErr = Math.hypot(last.x - truth.x, last.y - truth.y);
const rawOdomErr = Math.hypot(odom.x - truth.x, odom.y - truth.y);
check('LocalizationNode: publishes a correction per good scan', corrections.length >= 28, `${corrections.length}`);
check('LocalizationNode: correction stays locked to truth while raw odom drifts',
  corrErr < 0.06 && corrErr < rawOdomErr * 0.25,
  `corr err ${(corrErr * 100).toFixed(1)} cm, raw odom err ${(rawOdomErr * 100).toFixed(1)} cm`);
check('LocalizationNode: match events carry a healthy score', events.filter((e) => e.type === 'match').slice(-1)[0]?.score > 0.7,
  `score ${events.filter((e) => e.type === 'match').slice(-1)[0]?.score?.toFixed(3)}`);

// --- odom carry-forward between scans ---------------------------------
{
  const before = node.getPose();
  bus.publish('odom', { x: odom.x + 0.2 * Math.cos(odom.theta), y: odom.y + 0.2 * Math.sin(odom.theta), theta: odom.theta });
  const after = node.getPose();
  const moved = Math.hypot(after.x - before.x, after.y - before.y);
  check('LocalizationNode: estimate carries forward with odom between scans', moved > 0.15 && moved < 0.25, `moved ${moved.toFixed(3)} m`);
}

// --- lost + relocalize -----------------------------------------------
{
  events.length = 0;
  // re-seed so the estimate is a plausible (slightly-off) prior, then blind it
  node.setPose({ x: truth.x + 0.15, y: truth.y - 0.1, theta: truth.theta + 0.05 });
  const noise = () => Math.random() * 3.8 + 0.1; // fits nothing
  for (let i = 0; i < 4; i++) bus.publish('scan', { angleMin: 0, angleIncrement: Math.PI / 60, rangeMin: 0.12, rangeMax: 4, ranges: Array.from({ length: 120 }, noise) });
  check('LocalizationNode: a run of unmatchable scans -> "lost"', events.some((e) => e.type === 'lost'), events.map((e) => e.type).join(','));
  const nBefore = corrections.length;
  // the widened relocalize search is throttled (every Nth scan), so send a
  // few good scans -- one lands on the throttle window and re-acquires.
  for (let i = 0; i < 5; i++) bus.publish('scan', scanAt(truth));
  check('LocalizationNode: good scans after lost -> "relocalized" + correction resumes',
    events.some((e) => e.type === 'relocalized') && corrections.length > nBefore,
    events.map((e) => `${e.type}:${e.score?.toFixed(2)}`).join(' '));
}

// --- setPose seeds the estimate ------------------------------------
{
  node.setPose({ x: 3.3, y: 1.4, theta: 1.0 });
  const p = node.getPose();
  check('LocalizationNode: setPose seeds the estimate', Math.abs(p.x - 3.3) < 1e-9 && Math.abs(p.theta - 1.0) < 1e-9);
  check('LocalizationNode: setPose emits an event', events.some((e) => e.type === 'set-pose'));
}

node.stop();
bus.close();
console.log(failures === 0 ? '\nall localization-node smoke checks passed' : `\n${failures} check(s) failed`);
process.exit(failures === 0 ? 0 : 1);
