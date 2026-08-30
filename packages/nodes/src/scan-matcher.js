// Correlative scan matching (roadmap.md Phase 9, step 4 / "9a").
//
// Given a static map, a prior pose (from odometry), and a laser scan,
// search a small window of pose offsets for the one that best lines the
// scan's endpoints up with the map's obstacles. This is what turns "map +
// odometry" into "map-relative localization" -- LocalizationNode (step 5)
// runs it every scan and feeds the correction into PoseFusionNode, exactly
// where a VPS fix would go.
//
// Likelihood field (Thrun/Burgard/Fox, "Probabilistic Robotics", 6.4): the
// map is pre-blurred into L(x,y) = exp(-d(x,y)^2 / 2 sigma^2), where d is
// the distance to the nearest obstacle cell. A scan's score at a candidate
// pose is the mean of L looked up at each beam's endpoint -- smooth, so a
// coarse-to-fine grid search converges without derivatives. Built from the
// WALL cells only (pass the wall mask), so furniture that moved between the
// iPhone scan and the drive doesn't drag the estimate around.
//
// Pure functions + a tiny amount of state (the field). No bus, no classes
// beyond the field object -- LocalizationNode does the wiring.

const EXP = Math.exp;

/**
 * Blur a static occupancy grid into a likelihood field. Only cells flagged
 * in `occupied` seed the field; everything within ~3 sigma of one gets a
 * Gaussian bump (max-combined), the rest stay 0.
 *
 * @param {{originX,originY,cellSize,cols,rows}} grid
 * @param {ArrayLike<boolean|number>} occupied - row-major, length cols*rows
 * @param {{sigmaM?:number}} [opts]
 * @returns {{ grid, field: Float32Array, sigmaM: number }}
 */
export function buildLikelihoodField(grid, occupied, { sigmaM = 0.06 } = {}) {
  const { cellSize: s, cols, rows } = grid;
  const field = new Float32Array(cols * rows);
  const kr = Math.max(1, Math.ceil((3 * sigmaM) / s));
  const twoSig2 = 2 * sigmaM * sigmaM;

  // precompute the kernel (only depends on |dc|,|dr|)
  const kern = new Float32Array((2 * kr + 1) * (2 * kr + 1));
  for (let dr = -kr; dr <= kr; dr++) {
    for (let dc = -kr; dc <= kr; dc++) {
      kern[(dr + kr) * (2 * kr + 1) + (dc + kr)] = EXP(-((dc * s) ** 2 + (dr * s) ** 2) / twoSig2);
    }
  }

  for (let row = 0; row < rows; row++) {
    for (let col = 0; col < cols; col++) {
      if (!occupied[row * cols + col]) continue;
      for (let dr = -kr; dr <= kr; dr++) {
        const nr = row + dr;
        if (nr < 0 || nr >= rows) continue;
        for (let dc = -kr; dc <= kr; dc++) {
          const nc = col + dc;
          if (nc < 0 || nc >= cols) continue;
          const v = kern[(dr + kr) * (2 * kr + 1) + (dc + kr)];
          const idx = nr * cols + nc;
          if (v > field[idx]) field[idx] = v;
        }
      }
    }
  }
  return { grid, field, sigmaM };
}

/**
 * Convenience: threshold a log-odds grid (MapNode's) into an occupied mask
 * and build the field from it. `wallOnly` keeps only strongly-occupied
 * cells (a slicemap wall prior seeds L=2.0, furniture 0.7 -- a threshold
 * between them drops furniture).
 */
export function likelihoodFieldFromLogOdds(grid, logOdds, { wallLogOdds = 1.2, sigmaM = 0.06 } = {}) {
  const occ = new Uint8Array(logOdds.length);
  for (let i = 0; i < logOdds.length; i++) occ[i] = logOdds[i] >= wallLogOdds ? 1 : 0;
  return buildLikelihoodField(grid, occ, { sigmaM });
}

/**
 * Beam ranges -> endpoint offsets in the robot frame ([x forward, y left]),
 * dropping invalid / no-return beams. Precompute once per scan, reuse across
 * all the candidate poses.
 */
export function scanToPoints(scan, { beamStride = 1 } = {}) {
  const { angleMin = 0, angleIncrement, rangeMin = 0, rangeMax } = scan;
  const noReturnAt = rangeMax - 1e-6;
  const pts = [];
  for (let i = 0; i < scan.ranges.length; i += beamStride) {
    const r = scan.ranges[i];
    if (!(r > rangeMin) || r >= noReturnAt) continue;
    const a = angleMin + i * angleIncrement;
    pts.push(r * Math.cos(a), r * Math.sin(a)); // flat [x0,y0, x1,y1, ...]
  }
  return pts;
}

/**
 * Mean likelihood of a scan (robot-frame points from scanToPoints) placed
 * at pose (px,py,pth). 0..1; higher = the scan lines up with the map.
 */
export function scoreScan(lf, px, py, pth, pts) {
  const { originX, originY, cellSize, cols, rows } = lf.grid;
  const field = lf.field;
  const c = Math.cos(pth);
  const s = Math.sin(pth);
  let sum = 0;
  const n = pts.length >> 1;
  if (n === 0) return 0;
  for (let k = 0; k < pts.length; k += 2) {
    const lx = pts[k];
    const ly = pts[k + 1];
    const wx = px + c * lx - s * ly;
    const wy = py + s * lx + c * ly;
    const col = ((wx - originX) / cellSize) | 0;
    const row = ((wy - originY) / cellSize) | 0;
    if (col >= 0 && col < cols && row >= 0 && row < rows) sum += field[row * cols + col];
  }
  return sum / n;
}

function gridSearch(lf, pts, cx, cy, cth, halfX, halfTh, stepXY, stepTh) {
  let best = { x: cx, y: cy, theta: cth, score: -1 };
  let evals = 0;
  const nXY = Math.max(0, Math.round(halfX / stepXY));
  const nTh = Math.max(0, Math.round(halfTh / stepTh));
  for (let it = -nTh; it <= nTh; it++) {
    const th = cth + it * stepTh;
    for (let iy = -nXY; iy <= nXY; iy++) {
      const y = cy + iy * stepXY;
      for (let ix = -nXY; ix <= nXY; ix++) {
        const x = cx + ix * stepXY;
        const sc = scoreScan(lf, x, y, th, pts);
        evals++;
        if (sc > best.score) best = { x, y, theta: th, score: sc };
      }
    }
  }
  return { best, evals };
}

/**
 * Coarse-to-fine correlative scan match.
 * @param {{grid,field,sigmaM}} lf
 * @param {{x,y,theta}} priorPose
 * @param {object} scan - LaserScan-like
 * @param {object} [opts]
 * @returns {{ pose:{x,y,theta}, score:number, evals:number }}
 */
export function matchScan(lf, priorPose, scan, opts = {}) {
  const {
    windowM = 0.25, windowRad = 0.14,
    coarseM = 0.05, coarseRad = 0.035,
    fineM = 0.01, fineRad = 0.006,
    beamStride = 2,
  } = opts;
  const pts = scanToPoints(scan, { beamStride });
  if (pts.length === 0) return { pose: { ...priorPose }, score: 0, evals: 0 };

  const c = gridSearch(lf, pts, priorPose.x, priorPose.y, priorPose.theta, windowM, windowRad, coarseM, coarseRad);
  const f = gridSearch(lf, pts, c.best.x, c.best.y, c.best.theta, coarseM, coarseRad, fineM, fineRad);
  return {
    pose: { x: f.best.x, y: f.best.y, theta: f.best.theta },
    score: f.best.score,
    evals: c.evals + f.evals,
  };
}
