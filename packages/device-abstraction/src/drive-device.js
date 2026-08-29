// createDriveDevice — wraps the differential-drive base behind the
// WoT-style action shape used elsewhere in this stack (architecture doc,
// Layer 03: properties/actions/events instead of raw protocol commands).
//
// The base is a Roboteq controller (former-motor-protocol.md): one !G
// command carries both channels, so this is modeled as a single "drive"
// action, not two independent per-wheel ones — honest to what a
// differential-drive base is. Channels come from the manifest.

import { encodeCommand } from '../../transport/src/roboteq.js';

// left/right are normalized [-1, 1] — what TeleopNode and the manual
// sliders produce. ±1 maps to ±1000 Roboteq units (= ±200 wheel RPM). The
// eventual real-units path (wheel rad/s → RPM ×60/2π → ÷200×1000) is in
// former-motor-protocol.md; kept normalized here to match the existing
// teleop pipeline.
function toUnits(v) {
  return Math.max(-1000, Math.min(1000, Math.round(v * 1000)));
}

export function createDriveDevice(transport, manifest) {
  if (!manifest.drive) throw new Error('manifest has no "drive" entry');
  const chL = manifest.drive.channels?.left ?? 1;
  const chR = manifest.drive.channels?.right ?? 2;

  return {
    // Motors come up disabled on the controller (and after any E-STOP or
    // watchdog trip); enable() releases them. Call it once after connect.
    async enable() {
      await transport.send(encodeCommand('!MG'));
    },
    async estop() {
      await transport.send(encodeCommand('!EX'));
    },
    async setVelocity(left, right) {
      await transport.send(encodeCommand(`!G ${chL} ${toUnits(left)}_!G ${chR} ${toUnits(right)}`));
    },
    // velocity readback (manifest drive.readback.encoder = "?C") is still
    // TODO — needs a poll loop deriving wheel speed from ?C count deltas.
  };
}
