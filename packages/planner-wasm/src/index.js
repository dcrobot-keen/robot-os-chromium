// Loads pathfinder's grid package (GridAStar / HybridAStar), compiled to
// WASM by the sibling `pathfinder` project (see ../../../../pathfinder --
// this repo doesn't share a filesystem path with it on purpose, same as
// firmware/roboteq.js vs. transport/roboteq.js; vendor/ here is a vendored
// *build output*, refreshed with scripts/refresh-vendor.mjs when pathfinder's
// grid package changes, not source this repo maintains).
//
// Why WASM instead of a server call: pathfinder already runs this exact,
// tested code (pathfinder/pathfinder/grid, 30+ Go tests) behind an HTTP API
// at pathfinder/pathfinder/server, but requiring that server to be running
// alongside the robot stack would break ros-chromium's "the browser is the
// brain, nothing else to install" model (research.md). Compiling the same
// package to WASM gets the identical, already-verified pathfinding logic
// without a network hop or a second process.
import '../vendor/wasm_exec.js'; // side-effect only: defines globalThis.Go

const DEFAULT_WASM_URL = new URL('../vendor/pathfinder.wasm', import.meta.url);

let cached = null;

/**
 * @param {object} [options]
 * @param {URL|string} [options.wasmUrl] - defaults to the vendored build.
 * @param {(url: URL|string) => Promise<ArrayBuffer|Uint8Array>} [options.loadBytes]
 *   - defaults to `fetch(url).then(r => r.arrayBuffer())`, which needs a real
 *     HTTP(S) URL. Node test scripts have no server to fetch from, so they
 *     pass a `readFile`-based loader instead (see scripts/planner-wasm-smoke.mjs).
 * @returns {Promise<{ findPath(request: object): { path: number[][], distance: number } }>}
 */
export async function loadPlanner({ wasmUrl = DEFAULT_WASM_URL, loadBytes } = {}) {
  if (cached) return cached;

  const bytes = loadBytes
    ? await loadBytes(wasmUrl)
    : await fetch(wasmUrl).then((res) => res.arrayBuffer());

  const go = new globalThis.Go();
  const ready = new Promise((resolve) => {
    globalThis.__pathfinderWasmReady = resolve;
  });
  const { instance } = await WebAssembly.instantiate(bytes, go.importObject);
  go.run(instance); // fire-and-forget: Go's main() blocks on select{} and never resolves this
  await ready; // resolved by main.go right after it registers pathfinderFindPath

  cached = {
    /**
     * @param {object} request - see pathfinder/pathfinder/wasm/main.go's doc
     *   comment for the exact shape (occupied|blocks, start, goal, algorithm).
     * @returns {{ path: number[][], distance: number }}
     * @throws if pathfinderFindPath returns an { error } (e.g. no path found,
     *   start/goal inside an obstacle) -- see that Go doc comment.
     */
    findPath(request) {
      const result = globalThis.pathfinderFindPath(request);
      if (result.error) throw new Error(result.error);
      return { path: result.path, distance: result.distance };
    },
  };
  return cached;
}
