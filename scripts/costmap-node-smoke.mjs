// Smoke test for CostmapNode (roadmap.md Phase 9, step 6): merges a hard
// static wall layer with MapNode's live map. Walls stay solid even if
// MapNode's log-odds for them erode; a moved chair clears; a new obstacle
// the robot sees gets added.
//
//   node scripts/costmap-node-smoke.mjs
import { readFile } from 'node:fs/promises';
import {
  gridConfigFromBounds, cellIndex,
  MapNode, CostmapNode,
} from '@ros-chromium/nodes';
import { loadPlanner } from '@ros-chromium/planner-wasm';
import { LocalBus } from '@ros-chromium/bus';

let failures = 0;
const check = (name, cond, extra = '') => {
  console.log(`${cond ? 'PASS' : 'FAIL'}  ${name}${extra ? '  (' + extra + ')' : ''}`);
  if (!cond) failures++;
};
const wait = (ms) => new Promise((r) => setTimeout(r, ms));

// --- 4 x 4 m box; static layer = the perimeter walls only ---------------
const W = 4, H = 4;
const grid = gridConfigFromBounds({ minX: 0, minY: 0, maxX: W, maxY: H }, 0.1);
const n = grid.cols * grid.rows;
const at = (arr, wx, wy) => arr[cellIndex(grid, Math.floor((wx - grid.originX) / grid.cellSize), Math.floor((wy - grid.originY) / grid.cellSize))];
const setCell = (arr, wx, wy, v) => { arr[cellIndex(grid, Math.floor((wx - grid.originX) / grid.cellSize), Math.floor((wy - grid.originY) / grid.cellSize))] = v; };

const staticWalls = new Uint8Array(n);
for (let row = 0; row < grid.rows; row++) {
  for (let col = 0; col < grid.cols; col++) {
    const x = grid.originX + (col + 0.5) * grid.cellSize;
    const y = grid.originY + (row + 0.5) * grid.cellSize;
    if (x < 0.1 || x > W - 0.1 || y < 0.1 || y > H - 0.1) staticWalls[cellIndex(grid, col, row)] = 1;
  }
}

const bus = new LocalBus('costmap-smoke');
const costmaps = [];
bus.subscribe('costmap', (c) => costmaps.push(c));

// MapNode WITHOUT a prior (pure live layer). A slicemap prior on the wall
// column at x~0.5 to exercise "static wall stays solid even as MapNode's
// own belief in a nearby wall erodes".
const mapNode = new MapNode(bus, { scanTopic: 'scan', poseTopic: 'pose', mapTopic: 'map', grid, publishHz: 50 });
// seed a "furniture" blob at (2,2) and a "wall" cell at (0.55,2) via the
// slicemap-v1 prior path
{
  const codes = new Uint8Array(n); // 0 = unknown
  setCell(codes, 2, 2, 2);         // OCC_FURNITURE
  setCell(codes, 0.55, 2, 3);      // OCC_WALL (near, but not on, the static perimeter)
  mapNode.load({
    format: 'slicemap-v1', z: 0.18, band: 0.05, resolution: grid.cellSize,
    origin: [grid.originX, grid.originY], cols: grid.cols, rows: grid.rows,
    data: Buffer.from(codes).toString('base64'),
  });
}

const node = new CostmapNode(bus, {
  mapTopic: 'map', costmapTopic: 'costmap',
  staticOccupied: staticWalls, grid,
  inflationRadius: 0.15, dynThreshold: 0.55,
});

bus.publish('pose', { x: 2, y: 2, theta: 0 });
mapNode._publish(); // force one
await wait(60);

let cm = costmaps.at(-1);
check('CostmapNode: publishes on each MapNode update', costmaps.length >= 1);
check('CostmapNode: perimeter walls are occupied (static layer)',
  at(cm.occupied, 0.05, 2) && at(cm.occupied, 3.95, 2) && at(cm.occupied, 2, 0.05));
check('CostmapNode: the slicemap furniture blob shows as occupied (via MapNode live prob)',
  at(cm.occupied, 2, 2) === true, `staticCount ${cm.staticCount}, dynamicCount ${cm.dynamicCount}`);
check('CostmapNode: dynamicCount counts non-static occupied cells', cm.dynamicCount > 0);

// --- a chair that moved: free scans through (2,2) heal MapNode -> costmap clears there ---
bus.publish('pose', { x: 0.6, y: 2, theta: 0 }); // stand near the left wall, look right
for (let i = 0; i < 12; i++) {
  bus.publish('scan', { angleMin: 0, angleIncrement: 0, rangeMin: 0.02, rangeMax: 3.5, ranges: [3.2] }); // sweep straight across
}
mapNode._publish();
await wait(40);
cm = costmaps.at(-1);
check('CostmapNode: a healed live cell (moved chair) clears from the costmap', at(cm.occupied, 2, 2) === false);

// --- the static wall the sweep passed "through" stays solid ---
check('CostmapNode: a static wall cell stays occupied even if MapNode eroded its own belief',
  at(cm.occupied, 0.05, 2) === true);

// --- a new obstacle the robot sees: a beam that STOPS at (3.0, 2.0) ---
bus.publish('pose', { x: 1.0, y: 2.0, theta: 0 });
for (let i = 0; i < 8; i++) {
  bus.publish('scan', { angleMin: 0, angleIncrement: 0, rangeMin: 0.02, rangeMax: 3.5, ranges: [2.0] }); // hit at x=3.0
}
mapNode._publish();
await wait(40);
cm = costmaps.at(-1);
check('CostmapNode: a newly-seen obstacle (not in the static layer) is added', at(cm.occupied, 3.0, 2.0) === true);

// --- grid mismatch throws ---
check('CostmapNode: rejects a MapNode grid that does not match the static layer',
  (() => {
    try {
      new CostmapNode(bus, { mapTopic: 'm2', costmapTopic: 'c2', staticOccupied: new Uint8Array(n + 5), grid });
      return false;
    } catch { return true; }
  })());

// --- the costmap plans through planner-wasm ---
try {
  const planner = await loadPlanner({ loadBytes: (url) => readFile(url) });
  const req = {
    originX: cm.originX, originY: cm.originY, cellSize: cm.cellSize, cols: cm.cols, rows: cm.rows,
    occupied: cm.occupiedInflated, algorithm: 'gridastar',
  };
  const ok = planner.findPath({ ...req, start: { x: 1, y: 1 }, goal: { x: 1, y: 3 } });
  check('CostmapNode -> planner: a path exists inside the box', Array.isArray(ok.path) && ok.path.length > 0, `${ok.path?.length} pts`);
  let threw = null;
  try { planner.findPath({ ...req, start: { x: 1, y: 1 }, goal: { x: 5, y: 1 } }); } catch (e) { threw = e; }
  check('CostmapNode -> planner: a goal outside the walls is rejected', threw instanceof Error);
} catch (e) {
  check('CostmapNode -> planner: (wasm load)', false, e.message);
}

node.stop();
mapNode.stop();
bus.close();
console.log(failures === 0 ? '\nall costmap-node smoke checks passed' : `\n${failures} check(s) failed`);
process.exit(failures === 0 ? 0 : 1);
