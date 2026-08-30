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
//     wire units. setVelocityMps() converts real m/s to it via geometry.
//
// A drive.commands.* entry is either a string template (Roboteq ASCII line)
// or a structured object (TB3's OpenCR write/read op). fillSpec handles both:
// `${a.b}` refs in string leaves are resolved, and a leaf that is exactly
// one `${ref}` keeps the referenced value's type (so a numeric wire field
// stays a number, not "350").
//
// Optional readback: pass { readbackHz > 0 } and the device polls the
// manifest's drive.readback.* queries and exposes getState() — wheel
// velocity (m/s, from ?C count deltas), battery, current, temperature,
// faultFlags, estopButton. The reply parsing assumes Roboteq unit
// conventions (?V is tenths of a volt, ?DI 0 = pressed); a TB3 OpenCR
// manifest will want per-field handling once the real control table is
// known (todo-tb3.md).

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

const now = () => (typeof performance !== 'undefined' ? performance.now() : Date.now());

/** Top wheel speed (m/s) implied by geometry.maxWheelRpm + wheelRadius. */
export function maxWheelMps(geometry = {}) {
  if (!geometry.maxWheelRpm || !geometry.wheelRadius) return null;
  return ((geometry.maxWheelRpm * 2 * Math.PI) / 60) * geometry.wheelRadius;
}

/** Real m/s -> the stack's normalized [-1, 1], clamped. Needs maxWheelMps. */
export function mpsToNormalized(mps, geometry) {
  const max = maxWheelMps(geometry);
  if (!max) throw new Error('mpsToNormalized needs drive.geometry.maxWheelRpm and wheelRadius');
  return Math.max(-1, Math.min(1, mps / max));
}

// query string ("?V 2") or structured read op -> the reply key a decoder
// tags the answer with. Roboteq: first token after the leading punctuation
// ("?V 2" -> "V"). Structured: spec.reply ?? spec.key.
function replyKeyFor(spec) {
  if (typeof spec === 'string') return spec.replace(/^[?~!^]+/, '').trim().split(/\s+/)[0];
  return spec.reply ?? spec.key;
}

export function createDriveDevice(transport, manifest, { readbackHz = 0, onState } = {}) {
  const drive = manifest.drive;
  if (!drive || !drive.commands || !drive.commands.setVelocity) {
    throw new Error('manifest has no "drive.commands.setVelocity"');
  }
  const channels = drive.channels ?? { left: 1, right: 2 };
  const scale = drive.scale ?? 1000;
  const geometry = drive.geometry ?? {};
  const send = (spec) => transport.send(transport.encode(fillSpec(spec, vars())));

  // normalized [-1, 1] -> wire units, clamped
  const toUnits = (v) => Math.max(-scale, Math.min(scale, Math.round(v * scale)));

  let lastLeft = 0;
  let lastRight = 0;
  const vars = () => ({
    ch: channels,
    v: { left: toUnits(lastLeft), right: toUnits(lastRight) },
  });

  // --- readback state -------------------------------------------------
  const state = {
    counts: null, velocity: null, battery: null, current: null,
    temperature: null, faultFlags: null, estopButton: null, updatedAt: 0,
  };
  let lastEnc = null; // { counts:[l,r], t }
  const mPerCount = geometry.countsPerRev && geometry.wheelRadius
    ? (2 * Math.PI * geometry.wheelRadius) / geometry.countsPerRev
    : null;

  const readback = drive.readback ?? {};
  // field name (from the manifest) -> reply key -> how to fold it into state
  const foldByField = {
    encoder: (v) => {
      state.counts = { left: v[0], right: v[1] };
      const t = now();
      if (lastEnc && mPerCount) {
        const dt = (t - lastEnc.t) / 1000;
        if (dt > 1e-3) {
          state.velocity = {
            left: ((v[0] - lastEnc.counts[0]) * mPerCount) / dt,
            right: ((v[1] - lastEnc.counts[1]) * mPerCount) / dt,
          };
        }
      }
      lastEnc = { counts: [v[0], v[1]], t };
    },
    battery: (v) => { state.battery = v[0] / 10; },           // Roboteq ?V: tenths of a volt
    current: (v) => { state.current = { left: v[0] / 10, right: v[1] / 10 }; }, // ?A: tenths of an amp
    temperature: (v) => { state.temperature = v[0]; },
    faultFlags: (v) => { state.faultFlags = v[0]; },
    estopButton: (v) => { state.estopButton = v[0] === 0; }, // ?DI first input: 0 = pressed
  };
  const foldByKey = {};
  const readbackSpecs = [];
  for (const [field, spec] of Object.entries(readback)) {
    if (!foldByField[field]) continue;
    foldByKey[replyKeyFor(spec)] = foldByField[field];
    readbackSpecs.push(spec);
  }

  let unsubMessage = null;
  let rbTimer = null;

  function startReadback() {
    if (rbTimer || readbackSpecs.length === 0 || readbackHz <= 0) return;
    if (!unsubMessage && typeof transport.onMessage === 'function') {
      const handler = (msg) => {
        if (!msg || msg.type !== 'reply') return;
        const fold = foldByKey[msg.key];
        if (!fold) return;
        fold(msg.values);
        state.updatedAt = Date.now();
        if (onState) onState({ ...state });
      };
      transport.onMessage(handler);
      unsubMessage = () => {}; // transport.onMessage has no unsubscribe; handler is idempotent
    }
    rbTimer = setInterval(() => {
      for (const spec of readbackSpecs) transport.send(transport.encode(spec)).catch?.(() => {});
    }, 1000 / readbackHz);
  }
  function stopReadback() {
    if (rbTimer) clearInterval(rbTimer);
    rbTimer = null;
  }
  if (readbackHz > 0) startReadback();

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
    /** Real units. Needs drive.geometry.maxWheelRpm + wheelRadius. */
    async setVelocityMps(leftMps, rightMps) {
      await this.setVelocity(mpsToNormalized(leftMps, geometry), mpsToNormalized(rightMps, geometry));
    },
    /** Latest readback (all null until readback is running and replies arrive). */
    getState: () => ({ ...state }),
    startReadback,
    stopReadback,
  };
}
