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

import { WebSocketTransport } from '../packages/transport/src/websocket-transport.js';
import { encodeFrame } from '../packages/transport/src/frame.js';
import { CMD } from '../packages/transport/src/commands.js';

const PORT = Number(process.env.SIM_PORT || 8765);

function log(msg) {
  console.log(`[client ${new Date().toISOString()}] ${msg}`);
}

function heartbeatFrame(seq) {
  const payload = new Uint8Array(2);
  new DataView(payload.buffer).setUint16(0, seq, true);
  return encodeFrame(CMD.HEARTBEAT, payload);
}

function velocityFrame(left, right) {
  const payload = new Uint8Array(8);
  const dv = new DataView(payload.buffer);
  dv.setFloat32(0, left, true);
  dv.setFloat32(4, right, true);
  return encodeFrame(CMD.SET_VELOCITY, payload);
}

const transport = new WebSocketTransport(`ws://127.0.0.1:${PORT}`);

transport.onFrame(({ cmd, payload }) => {
  if (cmd === CMD.HEARTBEAT) {
    const seq = new DataView(payload.buffer, payload.byteOffset, payload.byteLength).getUint16(0, true);
    log(`heartbeat ack seq=${seq}`);
  }
});

await transport.connect();
log(`connected to firmware sim on port ${PORT}`);

let seq = 0;
const heartbeatTimer = setInterval(() => transport.send(heartbeatFrame(seq++)), 100);

setTimeout(() => {
  transport.send(velocityFrame(0.5, 0.5));
  log('sent SET_VELOCITY left=0.5 right=0.5');
}, 150);

setTimeout(() => {
  log('simulating a crashed tab — exiting without closing the WebSocket, no goodbye');
  clearInterval(heartbeatTimer);
  log('exiting. check the firmware sim log for the watchdog ESTOP ~300ms from now.');
  process.exit(0); // deliberately not transport._ws.close() — a real crash doesn't get to say goodbye either
}, 350);
