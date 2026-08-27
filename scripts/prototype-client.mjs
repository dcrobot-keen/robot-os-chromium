// Simplest working prototype, now over WebSocket: proves the wire protocol,
// the 300ms watchdog/E-STOP, and the actual WebSocketTransport class work
// end to end. This script is a Node.js stand-in for a browser tab — see
// apps/dashboard/index.html for the real-browser version of the same test.
//
// Run in two terminals:
//   1) node ../firmware/sim/src/index.js
//   2) node scripts/prototype-client.mjs
//
// What it does: connects the way a browser tab does, heartbeats for a bit,
// sends one velocity command, then exits the process WITHOUT closing the
// WebSocket first — simulating a crashed tab (a clean tab close would send
// a WS close frame; a crash doesn't). This script cannot observe the
// motors stopping; that's the point. Proof is in the firmware sim's own
// log, which should show a watchdog ESTOP ~300ms after the last heartbeat,
// logged after this process has already exited.

import { WebSocketTransport, startHeartbeat } from '../packages/transport/src/index.js';
import { createDriveDevice } from '../packages/device-abstraction/src/drive-device.js';

const PORT = Number(process.env.SIM_PORT || 8765);
const manifest = { drive: { setVelocity: 'SET_VELOCITY', velocity: 'GET_ENCODER' } };

function log(msg) {
  console.log(`[client ${new Date().toISOString()}] ${msg}`);
}

const transport = new WebSocketTransport(`ws://127.0.0.1:${PORT}`);
const drive = createDriveDevice(transport, manifest);

await transport.connect();
log(`connected to firmware sim on port ${PORT}`);

const heartbeat = startHeartbeat(transport, {
  onGap: (gap) => log(`heartbeat send gap ${gap.toFixed(0)}ms`),
  onSendError: (err) => log(`heartbeat send failed: ${err.message || err}`),
});

setTimeout(async () => {
  await drive.setVelocity(0.5, 0.5);
  log('sent SET_VELOCITY left=0.5 right=0.5');
}, 150);

setTimeout(() => {
  log('simulating a crashed tab — exiting without closing the WebSocket, no goodbye');
  heartbeat.stop();
  log('exiting. check the firmware sim log for the watchdog ESTOP ~300ms from now.');
  process.exit(0); // deliberately not transport._ws.close() — a real crash doesn't get to say goodbye either
}, 350);
