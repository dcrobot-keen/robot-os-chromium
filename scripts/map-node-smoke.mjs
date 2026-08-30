// Smoke test for MapNode -- no browser, no simulator. Checks the grid
// geometry stays byte-compatible with pathfinder/grid/grid.go, the ray
// traversal + log-odds Bayes update turn a synthetic laser scan into the
// right walls, inflation dilates them by the body radius, a built map
// serialize/load round-trips, and the published object plans a path
// through @ros-chromium/planner-wasm (the whole point: MapNode's output is
// a drop-in for NewGridFromOccupancy).
//
//   node scripts/map-node-smoke.mjs
import { readFile } from 'node:fs/promises';
import {
  gridConfigFromBounds,
  worldToCell,
  cellIndex,
  castRayCells,
  probFromLogOdds,
  occupancyFromLogOdds,
  serializeMap,
  deserializeMap,
  inflateOccupancy,
  clearDisc,
  MapNode,
  parseSlicemap,
  slicemapGrid,
  slicemapToLogOdds,
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

// --- 4. log-odds Bayes update: probability, threshold, inertia ---
{
  check('probFromLogOdds: L=0 -> p=0.5 (unobserved)', Math.abs(probFromLogOdds(0) - 0.5) < 1e-12);
  check('probFromLogOdds: L>0 -> p>0.5, L<0 -> p<0.5', probFromLogOdds(1) > 0.5 && probFromLogOdds(-1) < 0.5);

  const lo = new Float32Array([0, 0.85, -0.4, 2.0, -2.0]);
  const occ = occupancyFromLogOdds(lo, { occThreshold: 0.5 });
  check('occupancyFromLogOdds: unobserved and free cells -> not occupied', occ[0] === false && occ[2] === false && occ[4] === false);
  check('occupancyFromLogOdds: one positive hit is above p=0.5', occ[1] === true);

  // inertia: 20 free observations then 1 spurious hit stays free;
  // 20 hits then 1 spurious pass-through stays occupied. (hit/miss tally
  // would have flipped both.)
  let freeCell = 0, wallCell = 0;
  const clampL = (v) => Math.max(-2.0, Math.min(3.5, v));
  for (let i = 0; i < 20; i++) { freeCell = clampL(freeCell - 0.4); wallCell = clampL(wallCell + 0.85); }
  freeCell = clampL(freeCell + 0.85); // one spurious return
  wallCell = clampL(wallCell - 0.4);  // one spurious pass-through
  check('log-odds inertia: a corridor seen free 20x survives one noisy return', probFromLogOdds(freeCell) < 0.5, `p=${probFromLogOdds(freeCell).toFixed(3)}`);
  check('log-odds inertia: a wall seen 20x survives one noisy pass-through', probFromLogOdds(wallCell) > 0.5, `p=${probFromLogOdds(wallCell).toFixed(3)}`);
}

// --- 4b. serializeMap / deserializeMap round-trip ---
{
  const cfg = { originX: -1, originY: -2, cellSize: 0.05, cols: 40, rows: 30 };
  const lo = new Float32Array(cfg.cols * cfg.rows);
  for (let i = 0; i < lo.length; i++) lo[i] = (i % 7) - 3 + (i % 3) * 0.1; // spread of values
  const blob = serializeMap(cfg, lo);
  check('serializeMap: tagged blob with base64 data', blob.format === 'mapnode-logodds-v1' && typeof blob.data === 'string');
  const { cfg: cfg2, logOdds: lo2 } = deserializeMap(blob);
  check('deserializeMap: grid config round-trips', JSON.stringify(cfg2) === JSON.stringify(cfg));
  const maxErr = Math.max(...[...lo].map((v, i) => Math.abs(v - lo2[i])));
  check('deserializeMap: log-odds round-trip within int8 quantization', maxErr <= 1 / 32 + 1e-6, `max err ${maxErr.toFixed(4)}`);
}

// --- 4c. slicemap-v1 prior: MapNode.load() seeds the log-odds (Phase 9) ---
{
  // 8x6 slicemap @ 0.1 m: a wall column at col 1, a furniture cell at (4,3),
  // free everywhere else, one unknown at (7,5).
  const cols = 8, rows = 6, res = 0.1;
  const codes = new Uint8Array(cols * rows).fill(1); // FREE
  for (let r = 0; r < rows; r++) codes[r * cols + 1] = 3; // OCC_WALL
  codes[3 * cols + 4] = 2; // OCC_FURNITURE
  codes[5 * cols + 7] = 0; // UNKNOWN
  const blob = {
    format: 'slicemap-v1', z: 0.18, band: 0.05, resolution: res,
    origin: [-0.5, -0.5], cols, rows,
    data: Buffer.from(codes).toString('base64'),
  };

  const slice = parseSlicemap(blob);
  check('parseSlicemap: frame + codes length', slice.cols === cols && slice.resolution === res && slice.codes.length === cols * rows);
  check('parseSlicemap: rejects non-slicemap', (() => { try { parseSlicemap({ format: 'x' }); return false; } catch { return true; } })());

  const cfg = slicemapGrid(slice);
  check('slicemapGrid: MapNode grid config shape', cfg.originX === -0.5 && cfg.cellSize === res && cfg.cols === cols && cfg.rows === rows);

  const prior = slicemapToLogOdds(slice);
  check('slicemapToLogOdds: wall > furniture > 0 > free', prior[0 * cols + 1] > prior[3 * cols + 4] && prior[3 * cols + 4] > 0 && prior[0] < 0);

  const bus = new LocalBus('slicemap-prior-smoke');
  const node = new MapNode(bus, { scanTopic: 'sc', poseTopic: 'po', mapTopic: 'mp', grid: cfg, publishHz: 50 });
  node.load(blob);
  const at = (arr, c, r) => arr[cellIndex(cfg, c, r)];
  const m = node.snapshot();
  check('MapNode.load(slicemap): wall cells come back occupied', at(m.occupied, 1, 0) === true && at(m.occupied, 1, 3) === true);
  check('MapNode.load(slicemap): free cells come back free', at(m.occupied, 5, 2) === false);
  check('MapNode.load(slicemap): furniture cell starts occupied (weak prior)', at(m.occupied, 4, 3) === true);

  // self-heal: robot at cell (6,3), one beam sweeping -x through the
  // furniture cell (4,3) and stopping just short of the wall column. The
  // weak furniture prior gives way to the free sweep; the wall column,
  // untouched by the beam, keeps its prior.
  bus.publish('po', { x: (6 + 0.5) * res - 0.5, y: (3 + 0.5) * res - 0.5, theta: Math.PI });
  const sweep = { angleMin: 0, angleIncrement: 0, rangeMin: 0.02, rangeMax: 3.5, ranges: [0.44] };
  for (let i = 0; i < 8; i++) bus.publish('sc', sweep);
  const m2 = node.snapshot();
  check('MapNode: live free scans heal the furniture prior', at(m2.occupied, 4, 3) === false, `p=${(m2.prob[cellIndex(cfg, 4, 3)] / 255).toFixed(2)}`);
  check('MapNode: the wall prior survives (beam stops short of it)', at(m2.occupied, 1, 3) === true, `p=${(m2.prob[cellIndex(cfg, 1, 3)] / 255).toFixed(2)}`);
  check('MapNode.load(slicemap): rejects a mismatched grid', (() => {
    try { node.load({ ...blob, cols: cols + 1 }); return false; } catch { return true; }
  })());
  node.stop();
  bus.close();
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
  for (let i = 0; i < 5; i++) bus.publish('scan', scan); // build up log-odds
  await wait(120); // let the publish timer fire

  const map = node.snapshot();
  check('MapNode: publishes on its timer', maps.length >= 1);
  check('MapNode: grid config passes through unchanged', map.cols === cfg.cols && map.rows === cfg.rows && map.originX === 0);
  check('MapNode: occupied array length is cols*rows', map.occupied.length === cfg.cols * cfg.rows);
  check('MapNode: publishes a prob grid (Uint8, p*255)', map.prob instanceof Uint8Array && map.prob.length === cfg.cols * cfg.rows);

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

  // --- Phase 8: save the built map, reset, reload, same walls ---
  const saved = node.serialize();
  node.reset();
  check('MapNode: reset() clears the map (walls gone)', node.snapshot().occupied.every((c) => c === false));
  node.load(saved);
  const reloaded = node.snapshot();
  check('MapNode: load() restores the walls', wallNear(reloaded.occupied, 4, 2) && wallNear(reloaded.occupied, 0, 2));
  check('MapNode: load() rejects a mismatched grid', (() => {
    try { node.load({ ...saved, cols: saved.cols + 1 }); return false; } catch { return true; }
  })());

  node.stop();
  bus.close();
} catch (err) {
  console.log(`FAIL  unexpected error: ${err.stack || err}`);
  failures++;
}

console.log(failures === 0 ? '\nall map-node smoke checks passed' : `\n${failures} check(s) failed`);
process.exit(failures === 0 ? 0 : 1);
