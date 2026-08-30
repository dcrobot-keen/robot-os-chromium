// CostmapNode — merge a hard static layer with MapNode's live map into the
// grid the planner actually uses (roadmap.md Phase 9, step 6). This is
// Nav2's costmap_2d idea: a StaticLayer + an ObstacleLayer, combined by
// "occupied if either says so", then inflated.
//
// Why not just plan on MapNode's grid:
//   - MapNode seeded with the iPhone slicemap prior self-heals (a chair
//     that moved erodes, a new obstacle fills in) -- good.
//   - but the wall prior can also erode if localization wobbles and a few
//     beams sweep "through" a wall, and then the planner would route
//     through it. Bad.
//   The fix: keep a FIXED static layer of wall cells that is never eroded,
//   and OR it over MapNode's live output. Walls stay solid; everything
//   dynamic (furniture, people, new boxes) rides on the live layer and
//   still clears when MapNode's Bayes filter clears it.
//
// staticOccupied is a bool/0-1 array the caller builds -- typically
// wallMaskFromSlicemap(slice) (walls only; furniture is dynamic by nature
// and handled by MapNode's prior + live scans).

import { inflateOccupancy, clearDisc } from './map-node.js';

export class CostmapNode {
  /**
   * @param {object} bus - LocalBus
   * @param {object} opts
   * @param {string} opts.mapTopic - MapNode's published live map ({..., prob})
   * @param {string} opts.costmapTopic - where the merged grid is published
   * @param {ArrayLike<number|boolean>} opts.staticOccupied - row-major, length cols*rows
   * @param {{originX,originY,cellSize,cols,rows}} opts.grid - must match mapTopic's grid + staticOccupied
   * @param {number} [opts.inflationRadius=0]
   * @param {number} [opts.robotClearRadius=inflationRadius]
   * @param {number} [opts.dynThreshold=0.6] - live prob (0..1) above this = a dynamic obstacle
   * @param {string} [opts.poseTopic] - if given, the pose cell is force-cleared in occupiedInflated
   * @param {(costmap:object)=>void} [opts.onUpdate]
   */
  constructor(bus, {
    mapTopic, costmapTopic, staticOccupied, grid,
    inflationRadius = 0, robotClearRadius, dynThreshold = 0.6,
    poseTopic, onUpdate,
  } = {}) {
    if (!mapTopic || !costmapTopic || !staticOccupied || !grid) {
      throw new Error('CostmapNode requires mapTopic, costmapTopic, staticOccupied, grid');
    }
    const n = grid.cols * grid.rows;
    if (staticOccupied.length !== n) {
      throw new Error(`CostmapNode: staticOccupied length ${staticOccupied.length} != cols*rows ${n}`);
    }
    this._bus = bus;
    this._costmapTopic = costmapTopic;
    this._cfg = {
      originX: grid.originX, originY: grid.originY, cellSize: grid.cellSize,
      cols: grid.cols, rows: grid.rows,
    };
    this._static = Uint8Array.from(staticOccupied, (v) => (v ? 1 : 0));
    this._inflationRadius = inflationRadius;
    this._robotClearRadius = robotClearRadius ?? inflationRadius;
    this._dynThreshold = dynThreshold;
    this._onUpdate = onUpdate;
    this._pose = null;
    this._staticCount = this._static.reduce((a, b) => a + b, 0);

    this._unsubPose = poseTopic ? bus.subscribe(poseTopic, (p) => { this._pose = p; }) : () => {};
    this._unsubMap = bus.subscribe(mapTopic, (m) => this._onMap(m));
  }

  _onMap(m) {
    const c = this._cfg;
    if (m.cols !== c.cols || m.rows !== c.rows
      || Math.abs(m.originX - c.originX) > 1e-9 || Math.abs(m.cellSize - c.cellSize) > 1e-9) {
      throw new Error("CostmapNode: MapNode's grid does not match the static layer's grid");
    }
    const n = c.cols * c.rows;
    const thr255 = this._dynThreshold * 255;
    const occupied = new Array(n);
    let dyn = 0;
    for (let i = 0; i < n; i++) {
      const live = m.prob ? m.prob[i] > thr255 : !!(m.occupied && m.occupied[i]);
      if (live && !this._static[i]) dyn++;
      occupied[i] = this._static[i] === 1 || live;
    }
    const occupiedInflated = inflateOccupancy(occupied, c, this._inflationRadius);
    if (this._pose) clearDisc(occupiedInflated, c, this._pose.x, this._pose.y, this._robotClearRadius);

    const costmap = {
      originX: c.originX, originY: c.originY, cellSize: c.cellSize, cols: c.cols, rows: c.rows,
      occupied, occupiedInflated,
      staticCount: this._staticCount, dynamicCount: dyn,
      updatedAt: Date.now(),
    };
    this._bus.publish(this._costmapTopic, costmap);
    if (this._onUpdate) this._onUpdate(costmap);
  }

  /** Swap the static layer (e.g. a freshly loaded map). */
  setStatic(staticOccupied) {
    const n = this._cfg.cols * this._cfg.rows;
    if (staticOccupied.length !== n) throw new Error('CostmapNode.setStatic: wrong length');
    this._static = Uint8Array.from(staticOccupied, (v) => (v ? 1 : 0));
    this._staticCount = this._static.reduce((a, b) => a + b, 0);
  }

  stop() {
    this._unsubPose();
    this._unsubMap();
  }
}
