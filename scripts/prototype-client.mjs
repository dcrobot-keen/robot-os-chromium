// Simplest working prototype over WebSocket: proves the Roboteq line
// protocol, the serial watchdog (RWD), and the actual WebSocketTransport /
// createDriveDevice / startHeartbeat code work end to end. This script is a
// Node.js stand-in for a browser tab — see apps/dashboard/index.html for
// the real-browser version.
//
// Run in two terminals:
//   1) node ../firmware/sim/src/index.js          (or: SIM_RWD_MS=400 node ...)
//   2) node scripts/prototype-client.mjs
//
// What it does: connects the way a tab does, enables the motors (!MG),
// keepalives ("!B 3 1") for a bit, sends one velocity command, then exits
// the process WITHOUT closing the WebSocket — simulating a crashed tab. It
// can't observe the motors stopping; that's the point. Proof is in the
// firmware sim's own log: an RWD watchdog stop ~1s (SIM_RWD_MS) after the
// last command, logged after this process has already exited.

import { readFile } from 'node:fs/promises';
import { WebSocketTransport, startHeartbeat } from '../packages/transport/src/index.js';
import { createDriveDevice } from '../packages/device-abstraction/src/drive-device.js';

const PORT = Number(process.env.SIM_PORT || 8765);
const manifest = JSON.parse(await readFile(new URL('../manifests/former.manifest.json', import.meta.url)));

function log(msg) {
  console.log(`[client ${new Date().toISOString()}] ${msg}`);
}

const transport = new WebSocketTransport(`ws://127.0.0.1:${PORT}`);
const drive = createDriveDevice(transport, manifest);

await transport.connect();
log(`connected to firmware sim on port ${PORT}`);

transport.onMessage((msg) => {
  if (msg.type === 'reply' && msg.key === 'FID') log(`controller: ${msg.raw}`);
});
await transport.send(new TextEncoder().encode('?FID\r'));
await drive.enable();
log('sent !MG (enable motors)');

const heartbeat = startHeartbeat(transport, {
  onGap: (gap) => log(`keepalive send gap ${gap.toFixed(0)}ms`),
  onSendError: (err) => log(`keepalive send failed: ${err.message || err}`),
});

setTimeout(async () => {
  await drive.setVelocity(0.5, 0.5);
  log('sent !G 1 500_!G 2 500 (setVelocity 0.5, 0.5)');
}, 150);

setTimeout(() => {
  log('simulating a crashed tab — exiting without closing the WebSocket, no goodbye');
  heartbeat.stop();
  log('exiting. check the firmware sim log for the RWD watchdog stop ~1s (SIM_RWD_MS) from now.');
  process.exit(0); // deliberately not transport.close() — a real crash doesn't get to say goodbye
}, 350);
