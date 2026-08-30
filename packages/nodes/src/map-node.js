// MapNode — LIDAR scan + pose -> occupancy grid (roadmap.md Phase 7/8).
//
// "Mapping with known poses" -- the easy half of SLAM. It does NOT localize;
// it takes whatever pose the bus gives it (OdometryNode's dead-reckoned
// `odom`, or PoseFusionNode's corrected estimate, or -- in a sim smoke test
// -- ground truth) and folds each scan into the grid from there.
//
// Phase 8: the per-cell estimate is a **log-odds** value, updated by the
// standard binary Bayes filter (Thrun/Burgard/Fox, "Probabilistic
// Robotics", occupancy grid mapping):
//
//   L(cell) += l_occ   for the beam's endpoint cell   (p_occ  > 0.5)
//   L(cell) += l_free  for every cell the beam passed  (p_free < 0.5)
//   L clamped to [logOdds.min, logOdds.max]   (bounded confidence, so the
//                                              map can still change later)
//   p(cell) = 1 - 1/(1 + e^L) ;  L = 0  <=>  p = 0.5  <=>  never observed
//
// vs. the Phase-7 hit/miss tally this replaces, the win is inertia: a wall
// seen 30 times isn't erased by one noisy pass-through, and a corridor seen
// free 30 times isn't turned into a wall by one spurious return. `reset()`,
// `serialize()` and `load()` let a map be cleared / saved / reloaded.
//
// The published message keeps the Phase-7 shape so PlannerNode and the
// dashboard are unchanged:
//   { originX, originY, cellSize, cols, rows,
//     occupied, occupiedInflated,   // row-major bool[], grid.go's CellAt/index convention
//     prob,                          // row-major Uint8Array, p*255 (for a grayscale view)
//     updatedAt }
// `occupied` is `p > threshold`; `occupiedInflated` is that dilated by the
// robot body radius (pathfinder's A* treats the robot as a point), with the
// current pose cell forced free so the planner never rejects the start.

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

// --- log-odds occupancy ----------------------------------------------

// p_occ 0.7 -> +0.847, p_free 0.4 -> -0.405; clamp keeps confidence bounded
// so a long-standing wall/corridor can still be revised when the world (or
// the robot's belief about its pose) changes.
export const LOGODDS_DEFAULTS = { occ: 0.85, free: -0.4, min: -2.0, max: 3.5, threshold: 0.5 };

const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);

/** log-odds -> probability of occupied. L=0 -> 0.5. */
export function probFromLogOdds(l) {
  return 1 - 1 / (1 + Math.exp(l));
}

/**
 * Threshold a log-odds grid to a boolean occupancy array.
 * `occThreshold` is a probability (default 0.5); never-observed cells (L=0,
 * p=0.5) are therefore free unless the threshold is pushed below 0.5.
 */
export function occupancyFromLogOdds(logOdds, { occThreshold = 0.5 } = {}) {
  const n = logOdds.length;
  const out = new Array(n);
  for (let i = 0; i < n; i++) out[i] = probFromLogOdds(logOdds[i]) > occThreshold;
  return out;
}

/** log-odds grid -> Uint8Array of p*255, for a grayscale render. */
export function probGridU8(logOdds) {
  const out = new Uint8Array(logOdds.length);
  for (let i = 0; i < logOdds.length; i++) {
    out[i] = Math.round(probFromLogOdds(logOdds[i]) * 255);
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
 * cell containing (x,y) is always cleared -- keeps the planner from
 * rejecting "start point is inside an obstacle" when the robot is parked
 * close enough to a wall that inflation bleeds onto it.
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

// --- serialize / load (save & reload a built map) --------------------

const SERIAL_FORMAT = 'mapnode-logodds-v1';
const SERIAL_SCALE = 32; // int8 quantization: covers L in ~[-3.97, 3.97]

const B64 = typeof btoa === 'function'
  ? { enc: (u8) => btoa(String.fromCharCode(...u8)), dec: (s) => Uint8Array.from(atob(s), (c) => c.charCodeAt(0)) }
  : { enc: (u8) => Buffer.from(u8).toString('base64'), dec: (s) => new Uint8Array(Buffer.from(s, 'base64')) };

/** @param {{originX,originY,cellSize,cols,rows}} cfg @param {Float32Array} logOdds */
export function serializeMap(cfg, logOdds) {
  const q = new Int8Array(logOdds.length);
  for (let i = 0; i < logOdds.length; i++) {
    q[i] = clamp(Math.round(logOdds[i] * SERIAL_SCALE), -127, 127);
  }
  return {
    format: SERIAL_FORMAT,
    originX: cfg.originX, originY: cfg.originY, cellSize: cfg.cellSize,
    cols: cfg.cols, rows: cfg.rows, scale: SERIAL_SCALE,
    data: B64.enc(new Uint8Array(q.buffer)),
    savedAt: Date.now(),
  };
}

/** @returns {{ cfg, logOdds: Float32Array }} */
export function deserializeMap(obj) {
  if (!obj || obj.format !== SERIAL_FORMAT) {
    throw new Error(`deserializeMap: not a ${SERIAL_FORMAT} object`);
  }
  const n = obj.cols * obj.rows;
  const q = new Int8Array(B64.dec(obj.data).buffer);
  if (q.length !== n) throw new Error(`deserializeMap: data length ${q.length} != cols*rows ${n}`);
  const logOdds = new Float32Array(n);
  for (let i = 0; i < n; i++) logOdds[i] = q[i] / (obj.scale || SERIAL_SCALE);
  return {
    cfg: { originX: obj.originX, originY: obj.originY, cellSize: obj.cellSize, cols: obj.cols, rows: obj.rows },
    logOdds,
  };
}

// --- the node -----------------------------------------------------------

export class MapNode {
  /**
   * @param {object} bus - LocalBus
   * @param {object} opts
   * @param {string} opts.scanTopic - LaserScan-like msgs
   *   { angleMin, angleIncrement, rangeMin, rangeMax, ranges:number[] }.
   * @param {string} opts.poseTopic - {x,y,theta} in the grid's world frame.
   * @param {string} opts.mapTopic - where the occupancy grid object is published.
   * @param {{originX,originY,cellSize,cols,rows}} opts.grid - see gridConfigFromBounds().
   * @param {number} [opts.inflationRadius=0]
   * @param {number} [opts.robotClearRadius=inflationRadius]
   * @param {{occ?,free?,min?,max?,threshold?}} [opts.logOdds] - binary-Bayes params,
   *   see LOGODDS_DEFAULTS.
   * @param {number} [opts.publishHz=2]
   * @param {(map:object)=>void} [opts.onUpdate]
   */
  constructor(bus, {
    scanTopic,
    poseTopic,
    mapTopic,
    grid,
    inflationRadius = 0,
    robotClearRadius,
    logOdds,
    publishHz = 2,
    onUpdate,
  } = {}) {
    if (!scanTopic || !poseTopic || !mapTopic || !grid) {
      throw new Error('MapNode requires scanTopic, poseTopic, mapTopic, grid');
    }
    this._bus = bus;
    this._mapTopic = mapTopic;
    this._cfg = {
      originX: grid.originX, originY: grid.originY, cellSize: grid.cellSize,
      cols: grid.cols, rows: grid.rows,
    };
    this._inflationRadius = inflationRadius;
    this._robotClearRadius = robotClearRadius ?? inflationRadius;
    this._lo = { ...LOGODDS_DEFAULTS, ...(logOdds || {}) };
    this._onUpdate = onUpdate;

    this._logOdds = new Float32Array(this._cfg.cols * this._cfg.rows);
    this._pose = null;
    this._dirty = false;

    this._unsubPose = bus.subscribe(poseTopic, (pose) => { this._pose = pose; });
    this._unsubScan = bus.subscribe(scanTopic, (scan) => this._integrate(scan));
    this._timer = setInterval(() => { if (this._dirty) this._publish(); }, 1000 / publishHz);
  }

  _bump(idx, delta) {
    this._logOdds[idx] = clamp(this._logOdds[idx] + delta, this._lo.min, this._lo.max);
  }

  _integrate(scan) {
    if (!this._pose || !scan || !Array.isArray(scan.ranges)) return;
    const cfg = this._cfg;
    const { x, y, theta } = this._pose;
    const { angleMin = 0, angleIncrement, rangeMin = 0, rangeMax } = scan;
    const noReturnAt = rangeMax - 1e-6;

    for (let i = 0; i < scan.ranges.length; i++) {
      const r = scan.ranges[i];
      if (!(r > rangeMin)) continue; // invalid / too-close beam
      const noReturn = r >= noReturnAt;
      const reach = noReturn ? rangeMax : r;
      const a = theta + angleMin + i * angleIncrement;
      const cells = castRayCells(cfg, x, y, x + reach * Math.cos(a), y + reach * Math.sin(a));

      const lastIdx = cells.length - 1;
      for (let k = 0; k < lastIdx; k++) {
        const { col, row } = cells[k];
        if (inBounds(cfg, col, row)) this._bump(cellIndex(cfg, col, row), this._lo.free);
      }
      const end = cells[lastIdx];
      if (inBounds(cfg, end.col, end.row)) {
        this._bump(cellIndex(cfg, end.col, end.row), noReturn ? this._lo.free : this._lo.occ);
      }
    }
    this._dirty = true;
  }

  /** Build the current occupancy grid message without waiting for the timer. */
  snapshot() {
    const cfg = this._cfg;
    const occupied = occupancyFromLogOdds(this._logOdds, { occThreshold: this._lo.threshold });
    const occupiedInflated = inflateOccupancy(occupied, cfg, this._inflationRadius);
    if (this._pose) {
      clearDisc(occupiedInflated, cfg, this._pose.x, this._pose.y, this._robotClearRadius);
    }
    return {
      originX: cfg.originX, originY: cfg.originY, cellSize: cfg.cellSize,
      cols: cfg.cols, rows: cfg.rows,
      occupied,
      occupiedInflated,
      prob: probGridU8(this._logOdds),
      updatedAt: Date.now(),
    };
  }

  _publish() {
    this._dirty = false;
    const map = this.snapshot();
    this._bus.publish(this._mapTopic, map);
    if (this._onUpdate) this._onUpdate(map);
  }

  /** A save blob: grid config + quantized log-odds (see serializeMap). */
  serialize() {
    return serializeMap(this._cfg, this._logOdds);
  }

  /**
   * Replace the current map with a saved one. The saved grid config must
   * match this node's (same origin/cellSize/cols/rows) -- otherwise the
   * cells wouldn't line up. Returns this.
   */
  load(saved) {
    const { cfg, logOdds } = deserializeMap(saved);
    const c = this._cfg;
    const same = cfg.cols === c.cols && cfg.rows === c.rows
      && Math.abs(cfg.originX - c.originX) < 1e-9 && Math.abs(cfg.originY - c.originY) < 1e-9
      && Math.abs(cfg.cellSize - c.cellSize) < 1e-12;
    if (!same) {
      throw new Error('MapNode.load: saved grid config does not match this node\'s grid');
    }
    this._logOdds.set(logOdds);
    this._dirty = true;
    return this;
  }

  /** Drop all accumulated evidence (e.g. on teleport / relocalization). */
  reset() {
    this._logOdds.fill(0);
    this._dirty = false;
  }

  stop() {
    clearInterval(this._timer);
    this._unsubPose();
    this._unsubScan();
  }
}
