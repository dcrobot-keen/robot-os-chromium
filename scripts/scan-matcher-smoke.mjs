// Smoke test for the correlative scan matcher (packages/nodes/src/scan-matcher.js).
// No browser, no simulator: builds a box room, rasterizes it to a grid,
// blurs it into a likelihood field, synthesizes a laser scan at a known
// pose by ray-casting the box, then checks matchScan() recovers that pose
// from an offset prior.
//
//   node scripts/scan-matcher-smoke.mjs
import {
  gridConfigFromBounds,
  cellIndex,
  buildLikelihoodField,
  likelihoodFieldFromLogOdds,
  scanToPoints,
  scoreScan,
  matchScan,
} from '@ros-chromium/nodes';

let failures = 0;
const check = (name, cond, extra = '') => {
  console.log(`${cond ? 'PASS' : 'FAIL'}  ${name}${extra ? '  (' + extra + ')' : ''}`);
  if (!cond) failures++;
};

// --- a 6 x 5 m box room --------------------------------------------------
const WALLS = [
  [0, 0, 6, 0], [6, 0, 6, 5], [6, 5, 0, 5], [0, 5, 0, 0], // outer box
  [3.5, 0, 3.5, 2.2], // a stub interior wall -> gives the match some yaw signal
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

function synthScan(pose, { count = 360, rangeMax = 3.5, rangeMin = 0.12 } = {}) {
  const inc = (2 * Math.PI) / count;
  const ranges = new Array(count);
  for (let i = 0; i < count; i++) {
    const d = raycast(pose.x, pose.y, pose.theta + i * inc, rangeMax);
    ranges[i] = Math.max(rangeMin, Math.min(rangeMax, d));
  }
  return { angleMin: 0, angleIncrement: inc, rangeMin, rangeMax, ranges };
}

// --- rasterize the box into an occupancy grid + likelihood field --------
const grid = gridConfigFromBounds({ minX: -0.4, minY: -0.4, maxX: 6.4, maxY: 5.4 }, 0.05);
const occ = new Uint8Array(grid.cols * grid.rows);
{
  // mark a cell occupied if its centre is within ~1 cell of any wall segment
  const near = (px, py) => {
    for (const [x1, y1, x2, y2] of WALLS) {
      const vx = x2 - x1, vy = y2 - y1;
      const len2 = vx * vx + vy * vy || 1e-9;
      let t = ((px - x1) * vx + (py - y1) * vy) / len2;
      t = Math.max(0, Math.min(1, t));
      const d2 = (px - (x1 + t * vx)) ** 2 + (py - (y1 + t * vy)) ** 2;
      if (d2 <= (grid.cellSize * 0.8) ** 2) return true;
    }
    return false;
  };
  for (let row = 0; row < grid.rows; row++) {
    for (let col = 0; col < grid.cols; col++) {
      const px = grid.originX + (col + 0.5) * grid.cellSize;
      const py = grid.originY + (row + 0.5) * grid.cellSize;
      if (near(px, py)) occ[cellIndex(grid, col, row)] = 1;
    }
  }
}
const lf = buildLikelihoodField(grid, occ, { sigmaM: 0.08 });
check('buildLikelihoodField: field is nonzero near a wall, zero in open space',
  lf.field[cellIndex(grid, Math.round((0 - grid.originX) / grid.cellSize), Math.round((2.5 - grid.originY) / grid.cellSize))] > 0.5
  && lf.field[cellIndex(grid, Math.round((3 - grid.originX) / grid.cellSize), Math.round((2.5 - grid.originY) / grid.cellSize))] === 0);

// --- score: a scan scores high at its true pose, low when shifted -------
const truth = { x: 2.0, y: 2.0, theta: 0.30 };
const scan = synthScan(truth);
const pts = scanToPoints(scan, { beamStride: 2 });
const sTrue = scoreScan(lf, truth.x, truth.y, truth.theta, pts);
const sOff = scoreScan(lf, truth.x + 0.4, truth.y - 0.3, truth.theta + 0.25, pts);
check('scoreScan: high at the true pose', sTrue > 0.7, `score ${sTrue.toFixed(3)}`);
check('scoreScan: much lower at a wrong pose', sOff < sTrue * 0.7, `true ${sTrue.toFixed(3)} vs off ${sOff.toFixed(3)}`);

// --- matchScan: recover the true pose from an offset prior --------------
for (const off of [
  { dx: 0.12, dy: -0.08, dth: 0.05 },
  { dx: -0.18, dy: 0.15, dth: -0.08 },
  { dx: 0.05, dy: 0.05, dth: 0.10 },
]) {
  const prior = { x: truth.x + off.dx, y: truth.y + off.dy, theta: truth.theta + off.dth };
  const t0 = Date.now();
  const { pose, score, evals } = matchScan(lf, prior, scan);
  const ms = Date.now() - t0;
  const posErr = Math.hypot(pose.x - truth.x, pose.y - truth.y);
  const angErr = Math.abs(Math.atan2(Math.sin(pose.theta - truth.theta), Math.cos(pose.theta - truth.theta)));
  check(
    `matchScan: recovers pose from prior off by (${off.dx},${off.dy},${off.dth}) — err ${(posErr * 100).toFixed(1)} cm / ${(angErr * 57.3).toFixed(1)}°`,
    posErr < 0.03 && angErr < 0.035 && score > 0.7,
    `${evals} evals, ${ms} ms, score ${score.toFixed(3)}`,
  );
}

// --- matchScan on a scan that does NOT fit -> low score ----------------
{
  const bogus = { ...scan, ranges: scan.ranges.map(() => 1.0) }; // a uniform 1 m ring fits nothing
  const { score } = matchScan(lf, truth, bogus);
  check('matchScan: a scan that fits nothing scores low', score < 0.5, `score ${score.toFixed(3)}`);
}

// --- likelihoodFieldFromLogOdds: threshold splits wall (2.0) / furniture (0.7) ---
{
  const g = gridConfigFromBounds({ minX: 0, minY: 0, maxX: 1, maxY: 1 }, 0.1); // 11x11
  const lo = new Float32Array(g.cols * g.rows);
  lo[cellIndex(g, 2, 2)] = 2.0; // wall
  lo[cellIndex(g, 7, 7)] = 0.7; // furniture
  const f = likelihoodFieldFromLogOdds(g, lo, { wallLogOdds: 1.2, sigmaM: 0.12 });
  check('likelihoodFieldFromLogOdds: wall seeds the field, furniture does not',
    f.field[cellIndex(g, 2, 2)] > 0.5 && f.field[cellIndex(g, 7, 7)] === 0);
}

console.log(failures === 0 ? '\nall scan-matcher smoke checks passed' : `\n${failures} check(s) failed`);
process.exit(failures === 0 ? 0 : 1);
