// Smoke test for MapNode -- no browser, no simulator. Checks the grid
// geometry stays byte-compatible with pathfinder/grid/grid.go, the ray
// traversal + hit/miss thresholding turn a synthetic laser scan into the
// right walls, inflation dilates them by the body radius, and the published
// object plans a path through @ros-chromium/planner-wasm (the whole point:
// MapNode's output is a drop-in for NewGridFromOccupancy).
//
//   node scripts/map-node-smoke.mjs
import { readFile } from 'node:fs/promises';
import {
  gridConfigFromBounds,
  worldToCell,
  cellIndex,
  castRayCells,
  thresholdOccupancy,
  inflateOccupancy,
  clearDisc,
  MapNode,
} from '@ros-chromium/nodes';
import { loadPlanner } from '@ros-chromium/planner-wasm';
import { LocalBus } from '@ros-chromium/bus';

let failures = 0;
function check(name, cond, extra = '') {
  console.log(`${cond ? 'PASS' : 'FAIL'}  ${name}${extra ? '  (' + extra + ')' : ''}`);
  if (!cond) failures++;
}
const wait = (ms) => new Promise((r) => setTimeout(r, ms));

// --- 1. gridConfigFromBounds: matches grid.go's Bounds (cols = int(w/cs)+1) ---
{
  const cfg = gridConfigFromBounds({ minX: 0, minY: 0, maxX: 10, maxY: 10 }, 0.5);
  check('gridConfigFromBounds: 10m / 0.5m + 1 = 21 cells', cfg.cols === 21 && cfg.rows === 21, `${cfg.cols}x${cfg.rows}`);
  const padded = gridConfigFromBounds({ minX: 0, minY: 0, maxX: 10, maxY: 10 }, 0.5, 1);
  check('gridConfigFromBounds: padding shifts origin and grows the grid', padded.originX === -1 && padded.cols === 25, `origin ${padded.originX}, cols ${padded.cols}`);
}

// --- 2. worldToCell / cellIndex: floored, row-major, origin at min corner ---
{
  const cfg = { originX: 0, originY: 0, cellSize: 0.5, cols: 21, rows: 21 };
  const { col, row } = worldToCell(cfg, 1.2, 2.7);
  check('worldToCell: (1.2, 2.7) on a 0.5m grid -> col 2, row 5', col === 2 && row === 5, `col ${col}, row ${row}`);
  check('cellIndex: row-major row*cols + col', cellIndex(cfg, 2, 5) === 5 * 21 + 2);
  const neg = worldToCell({ ...cfg, originX: -1, originY: -1 }, -0.4, -0.4);
  check('worldToCell: floors toward -inf on the low side', neg.col === 1 && neg.row === 1, `col ${neg.col}, row ${neg.row}`);
}

// --- 3. castRayCells: start cell first, end cell last, no gaps ---
{
  const cfg = { originX: 0, originY: 0, cellSize: 0.5, cols: 21, rows: 21 };
  const horiz = castRayCells(cfg, 0.25, 0.25, 2.25, 0.25);
  check('castRayCells: horizontal ray walks cols 0..4 on row 0', horiz.length === 5 && horiz[0].col === 0 && horiz[4].col === 4 && horiz.every((c) => c.row === 0), JSON.stringify(horiz));

  const dot = castRayCells(cfg, 0.25, 0.25, 0.4, 0.4);
  check('castRayCells: sub-cell segment -> the single containing cell', dot.length === 1 && dot[0].col === 0 && dot[0].row === 0);

  const diag = castRayCells(cfg, 0.25, 0.25, 2.75, 2.75);
  check('castRayCells: diagonal ray ends in the goal cell', diag.at(-1).col === 5 && diag.at(-1).row === 5, JSON.stringify(diag.at(-1)));
  check('castRayCells: diagonal walk has no jumps > 1 cell', diag.every((c, i) => i === 0 || Math.abs(c.col - diag[i - 1].col) + Math.abs(c.row - diag[i - 1].row) === 1));
}

// --- 4. thresholdOccupancy: minHits AND endpoint ratio ---
{
  const occ = thresholdOccupancy([3, 1, 0], [0, 0, 0], { minHits: 2, occRatio: 0.2 });
  check('thresholdOccupancy: 3 hits -> occupied, 1 hit -> below minHits, 0 -> unobserved free', occ[0] === true && occ[1] === false && occ[2] === false);
  const glancing = thresholdOccupancy([5], [100], { minHits: 2, occRatio: 0.2 });
  check('thresholdOccupancy: many pass-throughs vs few endpoints -> free (ratio gate)', glancing[0] === false);
}

// --- 5. inflateOccupancy: circular dilation by a metric radius ---
{
  const cfg = { originX: 0, originY: 0, cellSize: 0.1, cols: 11, rows: 11 };
  const occ = new Array(11 * 11).fill(false);
  occ[cellIndex(cfg, 5, 5)] = true;
  const inf = inflateOccupancy(occ, cfg, 0.2);
  check('inflateOccupancy: a cell 2 away (0.2m) is filled', inf[cellIndex(cfg, 3, 5)] === true && inf[cellIndex(cfg, 5, 7)] === true);
  check('inflateOccupancy: a cell 4 away (0.4m) is not', inf[cellIndex(cfg, 1, 5)] === false);
  check('inflateOccupancy: does not mutate the input', occ[cellIndex(cfg, 3, 5)] === false);
}

// --- 6. clearDisc: forces the robot footprint free even over inflated walls ---
{
  const cfg = { originX: 0, originY: 0, cellSize: 0.1, cols: 11, rows: 11 };
  const occ = new Array(11 * 11).fill(true);
  clearDisc(occ, cfg, 0.55, 0.55, 0.2);
  check('clearDisc: robot cell + disc cleared', occ[cellIndex(cfg, 5, 5)] === false && occ[cellIndex(cfg, 3, 5)] === false);
  check('clearDisc: outside the disc untouched', occ[cellIndex(cfg, 0, 0)] === true);
  const occ2 = new Array(11 * 11).fill(true);
  clearDisc(occ2, cfg, 0.55, 0.55, 0);
  check('clearDisc: radius 0 still clears exactly the containing cell', occ2[cellIndex(cfg, 5, 5)] === false && occ2[cellIndex(cfg, 4, 5)] === true);
}

// --- synthetic 360-beam scan of an axis-aligned room, robot inside ---
// range from an interior point to the walls of the box [0,W] x [0,H].
function roomScan(px, py, theta, W, H) {
  const count = 360;
  const angleIncrement = (2 * Math.PI) / count;
  const rangeMin = 0.12;
  const rangeMax = 3.5;
  const ranges = new Array(count);
  for (let i = 0; i < count; i++) {
    const a = theta + i * angleIncrement;
    const cos = Math.cos(a);
    const sin = Math.sin(a);
    const tx = cos > 1e-9 ? (W - px) / cos : cos < -1e-9 ? -px / cos : Infinity;
    const ty = sin > 1e-9 ? (H - py) / sin : sin < -1e-9 ? -py / sin : Infinity;
    let d = Math.min(tx, ty);
    if (d > rangeMax) d = rangeMax; // no return
    ranges[i] = Math.max(rangeMin, d);
  }
  return { angleMin: 0, angleIncrement, rangeMin, rangeMax, ranges };
}

// --- 7. MapNode end to end: scan -> published grid -> a planned path ---
try {
  const W = 4;
  const H = 4;
  const cfg = gridConfigFromBounds({ minX: 0, minY: 0, maxX: W, maxY: H }, 0.1);
  const bus = new LocalBus('map-node-smoke');
  const maps = [];
  bus.subscribe('map', (m) => maps.push(m));

  const node = new MapNode(bus, {
    scanTopic: 'scan',
    poseTopic: 'pose',
    mapTopic: 'map',
    grid: cfg,
    inflationRadius: 0.2,
    publishHz: 20,
  });

  bus.publish('pose', { x: 2, y: 2, theta: 0 });
  const scan = roomScan(2, 2, 0, W, H);
  for (let i = 0; i < 3; i++) bus.publish('scan', scan); // >= minHits
  await wait(120); // let the publish timer fire

  const map = node.snapshot();
  check('MapNode: publishes on its timer', maps.length >= 1);
  check('MapNode: grid config passes through unchanged', map.cols === cfg.cols && map.rows === cfg.rows && map.originX === 0);
  check('MapNode: occupied array length is cols*rows', map.occupied.length === cfg.cols * cfg.rows);

  const at = (arr, wx, wy) => {
    const { col, row } = worldToCell(cfg, wx, wy);
    return arr[cellIndex(cfg, col, row)];
  };
  // a beam endpoint landing exactly on a grid line (walls at multiples of
  // cellSize) can round to either bordering cell, so a wall check looks at
  // the 3x3 block around the nominal point -- which is also how you'd sanity
  // -check a real, noisy map.
  const wallNear = (arr, wx, wy) => {
    const { col, row } = worldToCell(cfg, wx, wy);
    for (let dr = -1; dr <= 1; dr++) for (let dc = -1; dc <= 1; dc++) {
      const c = col + dc, r = row + dr;
      if (c >= 0 && c < cfg.cols && r >= 0 && r < cfg.rows && arr[cellIndex(cfg, c, r)]) return true;
    }
    return false;
  };
  check('MapNode: room interior near the robot is free', at(map.occupied, 2, 2) === false && at(map.occupied, 2.5, 2.5) === false);
  check('MapNode: all four walls got mapped as occupied',
    wallNear(map.occupied, 4, 2) && wallNear(map.occupied, 0, 2) && wallNear(map.occupied, 2, 0) && wallNear(map.occupied, 2, 4),
    `E ${wallNear(map.occupied, 4, 2)} W ${wallNear(map.occupied, 0, 2)} S ${wallNear(map.occupied, 2, 0)} N ${wallNear(map.occupied, 2, 4)}`);
  check('MapNode: inflation thickens the wall inward (0.15m clear of x=4 is now blocked)',
    at(map.occupied, 3.85, 2) === false && at(map.occupiedInflated, 3.85, 2) === true);
  check('MapNode: the robot cell stays free in the inflated grid', at(map.occupiedInflated, 2, 2) === false);

  // --- the payoff: hand the inflated grid to the real planner ---
  const planner = await loadPlanner({ loadBytes: (url) => readFile(url) });
  const req = {
    originX: map.originX, originY: map.originY, cellSize: map.cellSize, cols: map.cols, rows: map.rows,
    occupied: map.occupiedInflated,
    algorithm: 'gridastar',
  };
  const ok = planner.findPath({ ...req, start: { x: 2, y: 2 }, goal: { x: 3.4, y: 2 } });
  check('MapNode -> planner: finds a path across the mapped room', Array.isArray(ok.path) && ok.path.length > 0, `${ok.path?.length} pts, ${ok.distance?.toFixed(2)}m`);

  let threw = null;
  try {
    planner.findPath({ ...req, start: { x: 2, y: 2 }, goal: { x: 3.92, y: 2 } }); // inside the inflated wall
  } catch (err) {
    threw = err;
  }
  check('MapNode -> planner: a goal inside the inflated wall is rejected, not crashed', threw instanceof Error, threw?.message);

  node.stop();
  bus.close();
} catch (err) {
  console.log(`FAIL  unexpected error: ${err.stack || err}`);
  failures++;
}

console.log(failures === 0 ? '\nall map-node smoke checks passed' : `\n${failures} check(s) failed`);
process.exit(failures === 0 ? 0 : 1);
