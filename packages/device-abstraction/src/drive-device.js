// createDriveDevice — turns the semantic drive action into wire commands.
//
// Device-agnostic: the command vocabulary comes entirely from the manifest
// (drive.commands + drive.channels + drive.scale). A different
// differential-drive base that speaks the same wire protocol is a new
// manifest file with no change to this code — which is the "only the
// manifest changes per robot" goal from plan.md Phase 3. See
// manifests/former.manifest.json and ../../../former-motor-protocol.md.
//
// What is NOT in the manifest, by design:
//   - the byte encoding (line terminator, framing) — that belongs to the
//     wire protocol, so it comes from the transport's codec via
//     transport.encode(), selected by manifest.transport.kind (codecs.js).
//     This file has no protocol import.
//   - the [-1, 1] normalized-velocity convention — a stack-wide contract
//     (what TeleopNode and the sliders produce). drive.scale maps it to
//     wire units. A real-units (m/s) API is a separate follow-up.
//
// A drive.commands.* entry is either a string template (Roboteq ASCII line)
// or a structured object (TB3's OpenCR write/read op). fillSpec handles both:
// `${a.b}` refs in string leaves are resolved, and a leaf that is exactly
// one `${ref}` keeps the referenced value's type (so a numeric wire field
// stays a number, not "350").

function resolve(ref, vars) {
  const val = ref.split('.').reduce((o, k) => (o == null ? o : o[k]), vars);
  if (val === undefined || val === null) {
    throw new Error(`drive command references "${ref}", which the manifest does not provide`);
  }
  return val;
}

function fillTemplate(tpl, vars) {
  const solo = /^\$\{([\w.]+)\}$/.exec(tpl);
  if (solo) return resolve(solo[1], vars); // keep the value's type
  return tpl.replace(/\$\{([\w.]+)\}/g, (_, ref) => String(resolve(ref, vars)));
}

function fillSpec(spec, vars) {
  if (typeof spec === 'string') return fillTemplate(spec, vars);
  if (Array.isArray(spec)) return spec.map((s) => fillSpec(s, vars));
  if (spec && typeof spec === 'object') {
    const out = {};
    for (const [k, v] of Object.entries(spec)) out[k] = fillSpec(v, vars);
    return out;
  }
  return spec;
}

export function createDriveDevice(transport, manifest) {
  const drive = manifest.drive;
  if (!drive || !drive.commands || !drive.commands.setVelocity) {
    throw new Error('manifest has no "drive.commands.setVelocity"');
  }
  const channels = drive.channels ?? { left: 1, right: 2 };
  const scale = drive.scale ?? 1000;
  const send = (spec) => transport.send(transport.encode(fillSpec(spec, vars())));

  // normalized [-1, 1] -> wire units, clamped
  const toUnits = (v) => Math.max(-scale, Math.min(scale, Math.round(v * scale)));

  let lastLeft = 0;
  let lastRight = 0;
  const vars = () => ({
    ch: channels,
    v: { left: toUnits(lastLeft), right: toUnits(lastRight) },
  });

  return {
    // enable/estop are optional — a controller that comes up live just
    // omits them from the manifest.
    async enable() {
      if (drive.commands.enable) await send(drive.commands.enable);
    },
    async estop() {
      if (drive.commands.estop) await send(drive.commands.estop);
    },
    async setVelocity(left, right) {
      lastLeft = left;
      lastRight = right;
      await send(drive.commands.setVelocity);
    },
    // velocity readback (drive.readback.encoder) is still TODO — a poll
    // loop deriving wheel speed from encoder-count deltas.
  };
}
