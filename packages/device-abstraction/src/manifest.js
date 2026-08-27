// loadManifest — fetches and parses a per-robot manifest JSON (architecture
// doc, Layer 03). Deliberately thin: no schema validation yet, just a
// clear error if the fetch itself fails. Browser-only (uses fetch against
// an http(s) URL, as served by scripts/serve-dashboard.mjs) — there's no
// Node-side use for this yet, so it hasn't been made to also read local
// files.

export async function loadManifest(url) {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`failed to load manifest ${url}: ${res.status} ${res.statusText}`);
  return res.json();
}
