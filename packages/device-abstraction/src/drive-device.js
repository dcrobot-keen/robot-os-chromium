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

// Fill ${a.b} references in a manifest command template from `vars`.
function fillTemplate(tpl, vars) {
  return tpl.replace(/\$\{([\w.]+)\}/g, (_, ref) => {
    const val = ref.split('.').reduce((o, k) => (o == null ? o : o[k]), vars);
    if (val === undefined || val === null) {
      throw new Error(`drive command template references "${ref}", which the manifest does not provide`);
    }
    return String(val);
  });
}

export function createDriveDevice(transport, manifest) {
  const drive = manifest.drive;
  if (!drive || !drive.commands || !drive.commands.setVelocity) {
    throw new Error('manifest has no "drive.commands.setVelocity"');
  }
  const channels = drive.channels ?? { left: 1, right: 2 };
  const scale = drive.scale ?? 1000;
  const send = (line) => transport.send(transport.encode(line));

  // normalized [-1, 1] -> wire units, clamped
  const toUnits = (v) => Math.max(-scale, Math.min(scale, Math.round(v * scale)));

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
      await send(fillTemplate(drive.commands.setVelocity, {
        ch: channels,
        v: { left: toUnits(left), right: toUnits(right) },
      }));
    },
    // velocity readback (drive.readback.encoder) is still TODO — a poll
    // loop deriving wheel speed from encoder-count deltas.
  };
}
