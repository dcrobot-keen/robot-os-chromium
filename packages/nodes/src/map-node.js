// MapNode — roadmap.md Phase 7's last un-built node: LIDAR scan + pose ->
// occupancy grid, in exactly the shape pathfinder's grid package consumes.
//
// "Mapping with known poses" -- the easy half of SLAM (roadmap.md's table:
// "알려진 pose + 스캔 → 점유격자"). It does NOT localize; it takes whatever
// pose the bus gives it (OdometryNode's dead-reckoned `odom`, or
// PoseFusionNode's corrected estimate, or -- in a sim smoke test -- ground
// truth) and rasterizes each scan from there.
//
// The output object is a drop-in for `@ros-chromium/planner-wasm`'s
// findPath request and for pathfinder's `grid.NewGridFromOccupancy`:
//
//   { originX, originY, cellSize, cols, rows, occupied, occupiedInflated, updatedAt }
//
// `occupied` / `occupiedInflated` are row-major boolean arrays of length
// cols*rows, indexed `row * cols + col`, with the origin at the min corner
// and `col = floor((x - originX) / cellSize)` -- byte-for-byte the
// convention in pathfinder/pathfinder/grid/grid.go (CellAt / index), so a
// path planned over this grid lands where MapNode thinks the walls are.
//
// Why two arrays:
//   - `occupied` is the raw thresholded map -- what the walls actually are.
//   - `occupiedInflated` is `occupied` dilated by the robot's body radius,
//     because pathfinder's A* treats the robot as a dimensionless point
//     (grid.go has no inflation step). PlannerNode requests should pass
//     `occupiedInflated` as `occupied`; the raw map is kept for display and
//     for Phase 8's Bayesian accumulation.
//
// Occupancy is a plain hit/miss tally per cell, not log-odds -- log-odds
// accumulation is Phase 8. A cell is "occupied" once it has been a beam
// endpoint enough times and endpoints outweigh pass-throughs by the
// configured ratio; everything else (including never-observed cells) is
// free, which is the safe default for a planner (unknown == traversable
// only matters until the robot's own scans fill the space in).

// --- grid geometry: keep every formula identical to pathfinder/grid/grid.go ---

/**
 * Grid config covering [minX,maxX]x[minY,maxY] plus `padding` m on each side.
 * Matches grid.go's Bounds(): `cols = int(width/cellSize) + 1`.
 * @returns {{originX:number, originY:number, cellSize:number, cols:number, rows:number}}
 */
export function gridConfigFromBounds({ minX, minY, maxX, maxY }, cellSize, padding = 0) {
  const originX = minX - padding;
  const originY = minY - padding;
  const width = maxX - minX + 2 * padding;
  const height = maxY - minY + 2 * padding;
  return {
    originX,
    originY,
    cellSize,
    cols: Math.max(1, Math.floor(width / cellSize) + 1),
    rows: Math.max(1, Math.floor(height / cellSize) + 1),
  };
}

/** World point -> cell col/row (floored, like grid.go's CellAt -- may be out of bounds). */
export function worldToCell(cfg, x, y) {
  return {
    col: Math.floor((x - cfg.originX) / cfg.cellSize),
    row: Math.floor((y - cfg.originY) / cfg.cellSize),
  };
}

/** Row-major index, matching grid.go's index(col,row) = row*Cols + col. */
export function cellIndex(cfg, col, row) {
  return row * cfg.cols + col;
}

function inBounds(cfg, col, row) {
  return col >= 0 && col < cfg.cols && row >= 0 && row < cfg.rows;
}

// --- ray traversal --------------------------------------------------------

/**
 * Every grid cell a segment from (x0,y0) to (x1,y1) passes through, start
 * cell first, end cell last (Amanatides & Woo "A Fast Voxel Traversal
 * Algorithm", 2D). Cells outside the grid are included in the walk (so a
 * beam leaving and re-entering the grid still traverses correctly) -- the
 * caller drops out-of-bounds cells when it writes.
 *
 * @returns {Array<{col:number,row:number}>}
 */
export function castRayCells(cfg, x0, y0, x1, y1) {
  const s = cfg.cellSize;
  const fx0 = (x0 - cfg.originX) / s;
  const fy0 = (y0 - cfg.originY) / s;
  const fx1 = (x1 - cfg.originX) / s;
  const fy1 = (y1 - cfg.originY) / s;

  let col = Math.floor(fx0);
  let row = Math.floor(fy0);
  const endCol = Math.floor(fx1);
  const endRow = Math.floor(fy1);

  const dx = fx1 - fx0;
  const dy = fy1 - fy0;
  const stepCol = dx > 0 ? 1 : dx < 0 ? -1 : 0;
  const stepRow = dy > 0 ? 1 : dy < 0 ? -1 : 0;

  // parametric distance (in units of t along the segment, t in [0,1]) to the
  // next cell boundary in each axis, and between successive boundaries.
  const tDeltaCol = dx !== 0 ? Math.abs(1 / dx) : Infinity;
  const tDeltaRow = dy !== 0 ? Math.abs(1 / dy) : Infinity;

  let tMaxCol =
    dx !== 0
      ? (dx > 0 ? Math.floor(fx0) + 1 - fx0 : fx0 - Math.floor(fx0)) * tDeltaCol
      : Infinity;
  let tMaxRow =
    dy !== 0
      ? (dy > 0 ? Math.floor(fy0) + 1 - fy0 : fy0 - Math.floor(fy0)) * tDeltaRow
      : Infinity;

  const cells = [{ col, row }];
  // Bound the walk: Manhattan cell distance + a small slack is always enough,
  // and guarantees termination even with floating-point boundary ties.
  const maxSteps = Math.abs(endCol - col) + Math.abs(endRow - row) + 2;
  for (let i = 0; i < maxSteps; i++) {
    if (col === endCol && row === endRow) break;
    if (tMaxCol < tMaxRow) {
      col += stepCol;
      tMaxCol += tDeltaCol;
    } else {
      row += stepRow;
      tMaxRow += tDeltaRow;
    }
    cells.push({ col, row });
    if (col === endCol && row === endRow) break;
  }
  return cells;
}

// --- occupancy from tallies --------------------------------------------

/**
 * Threshold hit/miss tallies into a boolean occupancy array.
 * A cell is occupied iff hits >= minHits AND hits/(hits+misses) >= occRatio.
 * Never-observed cells (hits+misses == 0) are free.
 */
export function thresholdOccupancy(hits, misses, { minHits = 2, occRatio = 0.2 } = {}) {
  const n = hits.length;
  const out = new Array(n);
  for (let i = 0; i < n; i++) {
    const h = hits[i];
    const total = h + misses[i];
    out[i] = h >= minHits && total > 0 && h / total >= occRatio;
  }
  return out;
}

/**
 * Dilate a boolean occupancy array by `radiusM` (metric, circular structuring
 * element) so a point-robot planner keeps the body clear of walls. Returns a
 * new array; input is not mutated.
 */
export function inflateOccupancy(occupied, cfg, radiusM) {
  if (!radiusM || radiusM <= 0) return occupied.slice();
  const out = occupied.slice();
  const rc = Math.ceil(radiusM / cfg.cellSize);
  const r2 = radiusM * radiusM;
  const s = cfg.cellSize;
  for (let row = 0; row < cfg.rows; row++) {
    for (let col = 0; col < cfg.cols; col++) {
      if (!occupied[cellIndex(cfg, col, row)]) continue;
      for (let dr = -rc; dr <= rc; dr++) {
        for (let dc = -rc; dc <= rc; dc++) {
          if ((dc * s) * (dc * s) + (dr * s) * (dr * s) > r2) continue;
          const nc = col + dc;
          const nr = row + dr;
          if (inBounds(cfg, nc, nr)) out[cellIndex(cfg, nc, nr)] = true;
        }
      }
    }
  }
  return out;
}

/**
 * Force every cell within `radiusM` of (x,y) free, in place. At least the
 * cell containing (x,y) is always cleared. This is how MapNode keeps the
 * planner from rejecting requests with "start point is inside an obstacle"
 * when the robot is parked close enough to a wall that inflation bleeds onto
 * it -- a real deployment wants the robot's own footprint treated as known
 * free space anyway.
 */
export function clearDisc(occupied, cfg, x, y, radiusM) {
  const { col: c0, row: r0 } = worldToCell(cfg, x, y);
  if (inBounds(cfg, c0, r0)) occupied[cellIndex(cfg, c0, r0)] = false;
  if (!radiusM || radiusM <= 0) return occupied;
  const rc = Math.ceil(radiusM / cfg.cellSize);
  const r2 = radiusM * radiusM;
  const s = cfg.cellSize;
  for (let dr = -rc; dr <= rc; dr++) {
    for (let dc = -rc; dc <= rc; dc++) {
      if ((dc * s) * (dc * s) + (dr * s) * (dr * s) > r2) continue;
      const nc = c0 + dc;
      const nr = r0 + dr;
      if (inBounds(cfg, nc, nr)) occupied[cellIndex(cfg, nc, nr)] = false;
    }
  }
  return occupied;
}

// --- the node -----------------------------------------------------------

export class MapNode {
  /**
   * @param {object} bus - LocalBus
   * @param {object} opts
   * @param {string} opts.scanTopic - LaserScan-like msgs:
   *   { angleMin, angleIncrement, rangeMin, rangeMax, ranges:number[] }.
   *   Beam i's world bearing is pose.theta + angleMin + i*angleIncrement;
   *   a range >= rangeMax means "no return" (clears space, marks no wall).
   * @param {string} opts.poseTopic - {x,y,theta} in the grid's world frame.
   *   The most recent pose seen is paired with each incoming scan.
   * @param {string} opts.mapTopic - where the occupancy grid object is published.
   * @param {{originX:number,originY:number,cellSize:number,cols:number,rows:number}} opts.grid
   *   - see gridConfigFromBounds().
   * @param {number} [opts.inflationRadius=0] - metres to dilate walls by for
   *   `occupiedInflated` (typically the robot body radius + a margin).
   * @param {number} [opts.robotClearRadius=inflationRadius] - metres around the
   *   current pose forced free in `occupiedInflated`.
   * @param {{minHits?:number, occRatio?:number}} [opts.threshold]
   * @param {number} [opts.publishHz=2] - max map-publish rate; a publish only
   *   happens when at least one scan has been integrated since the last one.
   * @param {(map:object)=>void} [opts.onUpdate] - called with each published map.
   */
  constructor(bus, {
    scanTopic,
    poseTopic,
    mapTopic,
    grid,
    inflationRadius = 0,
    robotClearRadius,
    threshold,
    publishHz = 2,
    onUpdate,
  } = {}) {
    if (!scanTopic || !poseTopic || !mapTopic || !grid) {
      throw new Error('MapNode requires scanTopic, poseTopic, mapTopic, grid');
    }
    this._bus = bus;
    this._mapTopic = mapTopic;
    this._cfg = {
      originX: grid.originX,
      originY: grid.originY,
      cellSize: grid.cellSize,
      cols: grid.cols,
      rows: grid.rows,
    };
    this._inflationRadius = inflationRadius;
    this._robotClearRadius = robotClearRadius ?? inflationRadius;
    this._threshold = threshold;
    this._onUpdate = onUpdate;

    const n = this._cfg.cols * this._cfg.rows;
    this._hits = new Uint16Array(n);
    this._misses = new Uint16Array(n);
    this._pose = null;
    this._dirty = false;

    this._unsubPose = bus.subscribe(poseTopic, (pose) => {
      this._pose = pose;
    });
    this._unsubScan = bus.subscribe(scanTopic, (scan) => this._integrate(scan));

    this._timer = setInterval(() => {
      if (this._dirty) this._publish();
    }, 1000 / publishHz);
  }

  _integrate(scan) {
    if (!this._pose || !scan || !Array.isArray(scan.ranges)) return;
    const cfg = this._cfg;
    const { x, y, theta } = this._pose;
    const { angleMin = 0, angleIncrement, rangeMin = 0, rangeMax } = scan;
    const noReturnAt = rangeMax - 1e-6;

    for (let i = 0; i < scan.ranges.length; i++) {
      const r = scan.ranges[i];
      if (!(r > rangeMin)) continue; // invalid / too-close beam: no information
      const noReturn = r >= noReturnAt;
      const reach = noReturn ? rangeMax : r;
      const a = theta + angleMin + i * angleIncrement;
      const ex = x + reach * Math.cos(a);
      const ey = y + reach * Math.sin(a);

      const cells = castRayCells(cfg, x, y, ex, ey);
      // every cell the beam crossed is free space...
      const lastIdx = cells.length - 1;
      for (let k = 0; k < lastIdx; k++) {
        const { col, row } = cells[k];
        if (inBounds(cfg, col, row)) this._misses[cellIndex(cfg, col, row)]++;
      }
      // ...and the final cell holds a wall, unless the beam simply ran out of
      // range (no return -> that last cell is just more free space).
      const end = cells[lastIdx];
      if (inBounds(cfg, end.col, end.row)) {
        const idx = cellIndex(cfg, end.col, end.row);
        if (noReturn) this._misses[idx]++;
        else this._hits[idx]++;
      }
    }
    this._dirty = true;
  }

  /** Build the current occupancy grid message without waiting for the timer. */
  snapshot() {
    const cfg = this._cfg;
    const occupied = thresholdOccupancy(this._hits, this._misses, this._threshold);
    let occupiedInflated = inflateOccupancy(occupied, cfg, this._inflationRadius);
    if (this._pose) {
      clearDisc(occupiedInflated, cfg, this._pose.x, this._pose.y, this._robotClearRadius);
    }
    return {
      originX: cfg.originX,
      originY: cfg.originY,
      cellSize: cfg.cellSize,
      cols: cfg.cols,
      rows: cfg.rows,
      occupied,
      occupiedInflated,
      updatedAt: Date.now(),
    };
  }

  _publish() {
    this._dirty = false;
    const map = this.snapshot();
    this._bus.publish(this._mapTopic, map);
    if (this._onUpdate) this._onUpdate(map);
  }

  /** Drop all accumulated evidence (e.g. on teleport / relocalization). */
  reset() {
    this._hits.fill(0);
    this._misses.fill(0);
    this._dirty = false;
  }

  stop() {
    clearInterval(this._timer);
    this._unsubPose();
    this._unsubScan();
  }
}
