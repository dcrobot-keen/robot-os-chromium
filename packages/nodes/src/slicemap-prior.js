// slicemap-v1 -> MapNode log-odds prior (roadmap.md Phase 9, step 3).
//
// A slicemap-v1 file (scan-to-map-studio/scripts/slice_map.py) is a 2D
// occupancy grid sliced from an iPhone LiDAR scan at a robot's LiDAR mount
// height. Instead of treating it as ground truth, we seed MapNode's binary
// Bayes filter with it as a *prior*: walls get a strong "occupied" bias,
// furniture a weak one (it moves between the scan and the drive), unknown
// stays 0. The robot's own live scans then Bayes-update it -- a chair
// that's since been moved self-heals as the robot drives through where it
// used to be; a new obstacle fills in. That is the literal implementation
// of "apply the iPhone map to reality".
//
// codes: 0 unknown, 1 free, 2 occupied (furniture / unspecified), 3 occupied-wall.
// Row-major, row 0 = min y -- same convention as MapNode's grid
// (cellIndex = row*cols + col, origin at the min corner), so no flip.

export const SLICE_CODE = { UNKNOWN: 0, FREE: 1, OCC_FURNITURE: 2, OCC_WALL: 3 };

// Prior log-odds per class. Wall well below MapNode's default clamp max
// (3.5) so a handful of live scans can still revise it; furniture barely
// above 0 so it's erased fast if it's not really there any more.
export const PRIOR_DEFAULTS = { wall: 2.0, furniture: 0.7, free: -1.0 };

const b64dec = typeof atob === 'function'
  ? (s) => Uint8Array.from(atob(s), (c) => c.charCodeAt(0))
  : (s) => new Uint8Array(Buffer.from(s, 'base64'));

/** True for the output of parseSlicemap (vs. the raw slicemap-v1 JSON). */
export function isParsedSlice(obj) {
  return !!(obj && obj.codes && obj.codes.length !== undefined && typeof obj.resolution === 'number' && obj.format === undefined);
}

/**
 * @param {object} obj - raw slicemap-v1 JSON, or an already-parsed slice (passed through)
 * @returns {{ z:number, band:number, resolution:number, origin:[number,number],
 *   cols:number, rows:number, codes:Uint8Array }}
 */
export function parseSlicemap(obj) {
  if (isParsedSlice(obj)) return obj;
  if (!obj || obj.format !== 'slicemap-v1') throw new Error('not a slicemap-v1 object');
  const codes = b64dec(obj.data);
  if (codes.length !== obj.cols * obj.rows) {
    throw new Error(`slicemap data length ${codes.length} != cols*rows ${obj.cols * obj.rows}`);
  }
  return {
    z: obj.z, band: obj.band, resolution: obj.resolution,
    origin: [obj.origin[0], obj.origin[1]], cols: obj.cols, rows: obj.rows, codes,
  };
}

/** slicemap frame -> a MapNode grid config. The prebuilt map defines the world frame. */
export function slicemapGrid(slice) {
  return {
    originX: slice.origin[0], originY: slice.origin[1], cellSize: slice.resolution,
    cols: slice.cols, rows: slice.rows,
  };
}

/**
 * slicemap codes -> a Uint8Array wall mask (1 where OCC_WALL), for
 * buildLikelihoodField() -- localization matches structure, not furniture.
 */
export function wallMaskFromSlicemap(slice) {
  const m = new Uint8Array(slice.codes.length);
  for (let i = 0; i < m.length; i++) m[i] = slice.codes[i] === SLICE_CODE.OCC_WALL ? 1 : 0;
  return m;
}

/**
 * slicemap codes -> a Float32Array of prior log-odds, one per cell, matching
 * MapNode's row-major indexing.
 * @param {object} slice - parseSlicemap() output
 * @param {{wall?:number, furniture?:number, free?:number}} [opts]
 */
export function slicemapToLogOdds(slice, opts = {}) {
  const { wall, furniture, free } = { ...PRIOR_DEFAULTS, ...opts };
  const { codes } = slice;
  const lo = new Float32Array(codes.length);
  for (let i = 0; i < codes.length; i++) {
    const c = codes[i];
    lo[i] = c === SLICE_CODE.OCC_WALL ? wall
      : c === SLICE_CODE.OCC_FURNITURE ? furniture
      : c === SLICE_CODE.FREE ? free
      : 0; // unknown
  }
  return lo;
}
