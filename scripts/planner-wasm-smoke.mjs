// Smoke test for @ros-chromium/planner-wasm + PlannerNode -- no browser
// needed (WebAssembly + BroadcastChannel are both available in Node 22+, and
// LocalBus already documents relying on that). Loads the vendored
// pathfinder.wasm build via Node's fs instead of fetch() (Node's built-in
// fetch doesn't support file: URLs -- see planner-wasm's loadBytes option,
// added for exactly this), then re-checks the same scenarios pathfinder's
// own Go test suite already covers (open space, routing around a wall,
// Hybrid A*, a fully-enclosed goal), plus PlannerNode's bus wiring end to end.
//
//   node scripts/planner-wasm-smoke.mjs
import { readFile } from 'node:fs/promises';
import { loadPlanner } from '@ros-chromium/planner-wasm';
import { PlannerNode } from '@ros-chromium/nodes';
import { LocalBus } from '@ros-chromium/bus';

let failures = 0;
function check(name, cond, extra = '') {
  console.log(`${cond ? 'PASS' : 'FAIL'}  ${name}${extra ? '  (' + extra + ')' : ''}`);
  if (!cond) failures++;
}

const baseGrid = { originX: 0, originY: 0, cellSize: 0.2, cols: 100, rows: 100 };

try {
  const t0 = Date.now();
  const planner = await loadPlanner({ loadBytes: (url) => readFile(url) });
  console.log(`planner-wasm loaded in ${Date.now() - t0}ms`);

  // --- open space: path should be close to the straight-line distance ---
  {
    const start = { x: 1, y: 1 };
    const goal = { x: 9, y: 9 };
    const straightLine = Math.hypot(goal.x - start.x, goal.y - start.y);
    const { distance } = planner.findPath({ ...baseGrid, blocks: [], start, goal, algorithm: 'gridastar' });
    check('open space: gridastar path ~= straight line', Math.abs(distance - straightLine) < 0.5, `got ${distance.toFixed(3)}`);
  }

  // --- wall between start and goal: must detour ---
  let wallPath;
  {
    const start = { x: 1, y: 5 };
    const goal = { x: 9, y: 5 };
    const straightLine = Math.hypot(goal.x - start.x, goal.y - start.y);
    const wall = [[[4.9, 3], [5.1, 3], [5.1, 7], [4.9, 7], [4.9, 3]]];
    const result = planner.findPath({ ...baseGrid, blocks: wall, start, goal, algorithm: 'gridastar' });
    wallPath = result.path;
    check('wall: gridastar detours', result.distance > straightLine + 0.5, `got ${result.distance.toFixed(3)}, straight line ${straightLine.toFixed(3)}`);
  }

  // --- same grid via a raw occupancy bitmap instead of blocks (NewGridFromOccupancy) ---
  {
    const { cols, rows } = baseGrid;
    const occupied = new Uint8Array(cols * rows);
    // mark the same wall (x in [4.9,5.1], y in [3,7]) as occupied cells directly
    for (let row = 0; row < rows; row++) {
      for (let col = 0; col < cols; col++) {
        const x = baseGrid.originX + (col + 0.5) * baseGrid.cellSize;
        const y = baseGrid.originY + (row + 0.5) * baseGrid.cellSize;
        if (x > 4.9 && x < 5.1 && y > 3 && y < 7) occupied[row * cols + col] = 1;
      }
    }
    const start = { x: 1, y: 5 };
    const goal = { x: 9, y: 5 };
    const result = planner.findPath({ ...baseGrid, occupied, start, goal, algorithm: 'gridastar' });
    check(
      'occupied bitmap: matches the equivalent blocks-based path length',
      wallPath && Math.abs(result.path.length - wallPath.length) <= 2,
      `bitmap path len ${result.path?.length}, blocks path len ${wallPath?.length}`
    );
  }

  // --- Hybrid A* still works through the same entry point ---
  {
    const start = { x: 1, y: 5 };
    const goal = { x: 9, y: 5 };
    const wall = [[[4.9, 3], [5.1, 3], [5.1, 7], [4.9, 7], [4.9, 3]]];
    const { path } = planner.findPath({ ...baseGrid, blocks: wall, start, goal, algorithm: 'hybridastar' });
    check('wall: hybridastar also finds a path', path.length > 0);
  }

  // --- fully enclosed goal: findPath must throw, not crash the process ---
  {
    const start = { x: 1, y: 1 };
    const goal = { x: 5, y: 5 };
    const box = [[[4, 4], [6, 4], [6, 6], [4, 6], [4, 4]]];
    let threw = null;
    try {
      planner.findPath({ ...baseGrid, blocks: box, start, goal, algorithm: 'gridastar' });
    } catch (err) {
      threw = err;
    }
    check('enclosed goal: findPath throws (not a crash)', threw instanceof Error, threw?.message);
  }

  // --- PlannerNode: request over the bus, path comes back over the bus ---
  {
    const bus = new LocalBus('planner-wasm-smoke');
    const node = new PlannerNode(bus, { requestTopic: 'plan-request', pathTopic: 'plan-result' });
    const start = { x: 1, y: 1 };
    const goal = { x: 9, y: 9 };
    const result = await new Promise((resolve) => {
      bus.subscribe('plan-result', resolve);
      bus.publish('plan-request', { requestId: 'req-1', ...baseGrid, blocks: [], start, goal, algorithm: 'gridastar' });
    });
    check('PlannerNode: request/response round-trips over the bus with matching requestId', result.requestId === 'req-1' && Array.isArray(result.path));
    node.stop();
    bus.close();
  }
} catch (err) {
  console.log(`FAIL  unexpected error: ${err.stack || err}`);
  failures++;
}

console.log(failures === 0 ? '\nall planner-wasm smoke checks passed' : `\n${failures} check(s) failed`);
process.exit(failures === 0 ? 0 : 1);
