// Simplest working prototype: proves the wire protocol and the 300ms
// watchdog/E-STOP work end to end, standing in for a browser tab until
// WebSerial + real hardware are wired up (plan.md Phase 1/2, research.md §2).
//
// Run in two terminals:
//   1) node ../firmware/sim/src/index.js
//   2) node scripts/prototype-client.mjs
//
// What it does: connects the way a browser tab eventually will, heartbeats
// for a bit, sends one velocity command, then destroys the connection
// WITHOUT a goodbye — simulating a crashed tab or a pulled USB cable — and
// exits immediately. This script cannot observe the motors stopping; that's
// the point. Proof is in the firmware sim's own log, which should show a
// watchdog ESTOP ~300ms after the last heartbeat, logged after this process
// has already exited.

import net from 'node:net';
import { encodeFrame, FrameDecoder } from '../packages/transport/src/frame.js';
import { CMD } from '../packages/transport/src/commands.js';

const HOST = '127.0.0.1';
const PORT = Number(process.env.SIM_PORT || 8765);

function log(msg) {
  console.log(`[client ${new Date().toISOString()}] ${msg}`);
}

function sendHeartbeat(socket, seq) {
  const payload = new Uint8Array(2);
  new DataView(payload.buffer).setUint16(0, seq, true);
  socket.write(encodeFrame(CMD.HEARTBEAT, payload));
}

function sendVelocity(socket, left, right) {
  const payload = new Uint8Array(8);
  const dv = new DataView(payload.buffer);
  dv.setFloat32(0, left, true);
  dv.setFloat32(4, right, true);
  socket.write(encodeFrame(CMD.SET_VELOCITY, payload));
  log(`sent SET_VELOCITY left=${left} right=${right}`);
}

const socket = net.connect(PORT, HOST, () => log(`connected to firmware sim at ${HOST}:${PORT}`));

const decoder = new FrameDecoder();
socket.on('data', (data) => {
  for (const { cmd, payload } of decoder.push(data)) {
    if (cmd === CMD.HEARTBEAT) {
      const seq = new DataView(payload.buffer, payload.byteOffset, payload.byteLength).getUint16(0, true);
      log(`heartbeat ack seq=${seq}`);
    }
  }
});
socket.on('error', (err) => log(`socket error: ${err.message}`));

let seq = 0;
const heartbeatTimer = setInterval(() => sendHeartbeat(socket, seq++), 100);

setTimeout(() => sendVelocity(socket, 0.5, 0.5), 150);

setTimeout(() => {
  log('simulating crashed tab / pulled cable — destroying connection, no goodbye');
  clearInterval(heartbeatTimer);
  socket.destroy();
  log('exiting. check the firmware sim log for the watchdog ESTOP ~300ms from now.');
  process.exit(0);
}, 350);
