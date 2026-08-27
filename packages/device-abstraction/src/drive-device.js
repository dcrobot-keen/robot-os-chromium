// createDriveDevice — wraps the differential-drive base's SET_VELOCITY
// command behind the WoT-style action shape used elsewhere in this stack
// (architecture doc, Layer 03: properties/actions/events instead of raw
// protocol frames).
//
// Course correction from the original architecture sketch: that doc
// modeled left and right wheels as two independent "motor" devices, each
// with its own setVelocity. The actual wire protocol (research.md) sends
// both wheels' targets in a single SET_VELOCITY frame, so two independent
// per-wheel actions can't be sent separately without one of them
// clobbering the other's last-known target. Modeling it as one "drive"
// action for both wheels is honest to what a differential-drive base
// actually is: one combined actuator, not two independent ones. See the
// note in ../../../manifests/rover.manifest.json.

import { encodeFrame } from '../../transport/src/frame.js';
import { CMD } from '../../transport/src/commands.js';

export function createDriveDevice(transport, manifest) {
  if (!manifest.drive) throw new Error('manifest has no "drive" entry');

  return {
    async setVelocity(leftMps, rightMps) {
      const payload = new Uint8Array(8);
      const dv = new DataView(payload.buffer);
      dv.setFloat32(0, leftMps, true);
      dv.setFloat32(4, rightMps, true);
      await transport.send(encodeFrame(CMD.SET_VELOCITY, payload));
    },
    // velocity readback (manifest's "velocity": "GET_ENCODER") is TODO —
    // GET_ENCODER isn't implemented by the firmware/simulator yet.
  };
}
