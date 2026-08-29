// Smoke test for apps/signaling-server + packages/rtc/SignalingClient —
// the parts of Phase 5 that run without a browser. It does NOT touch
// RTCPeerConnection (Node has none); it only proves the rendezvous:
// hello -> ready, peer-joined / peer-left, and that a `signal` blob from
// one peer comes out verbatim at the other. The SDP/ICE payloads here are
// fake strings — the server never looks inside them.
//
// Self-contained: spawns its own signaling-server on a throwaway port,
// runs the checks, prints PASS/FAIL, exits non-zero on failure.
//
//   node scripts/signaling-smoke.mjs

import { spawn } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { dirname, resolve } from 'node:path';
import { SignalingClient } from '../packages/rtc/src/signaling-client.js';

const here = dirname(fileURLToPath(import.meta.url));
const SERVER = resolve(here, '../apps/signaling-server/src/index.js');
const PORT = Number(process.env.SIGNALING_PORT || 9788);
const URL = `ws://127.0.0.1:${PORT}`;

const wait = (ms) => new Promise((r) => setTimeout(r, ms));
let failures = 0;
function check(name, cond) {
  console.log(`${cond ? 'PASS' : 'FAIL'}  ${name}`);
  if (!cond) failures++;
}
// Collect a callback's args until `settle` ms pass with no new call.
function recorder() {
  const calls = [];
  return { fn: (...a) => calls.push(a.length === 1 ? a[0] : a), calls };
}

const server = spawn(process.execPath, [SERVER], {
  env: { ...process.env, SIGNALING_PORT: String(PORT) },
  stdio: ['ignore', 'inherit', 'inherit'],
});
process.on('exit', () => server.kill());

try {
  await wait(400); // let the server bind

  // --- host registers into an empty room ---------------------------------
  const host = new SignalingClient(URL, { role: 'host', robot: 'rover-01', manifest: { robot: 'rover-01' } });
  const hostJoined = recorder();
  const hostLeft = recorder();
  const hostSignals = recorder();
  host.onPeerJoined(hostJoined.fn);
  host.onPeerLeft(hostLeft.fn);
  host.onSignal(hostSignals.fn);
  const hostReady = await host.connect();
  check('host ready with empty room', Array.isArray(hostReady.peers) && hostReady.peers.length === 0);

  // --- a second host for the same robot is rejected ---------------------
  const host2 = new SignalingClient(URL, { role: 'host', robot: 'rover-01' });
  let host2Err = null;
  try {
    await host2.connect();
  } catch (e) {
    host2Err = e;
  }
  check('second host for same robot rejected', host2Err && /already has a host/.test(host2Err.message));
  host2.close();

  // --- operator joins, both sides learn about each other ---------------
  const op = new SignalingClient(URL, { role: 'operator', robot: 'rover-01' });
  const opSignals = recorder();
  op.onSignal(opSignals.fn);
  const opReady = await op.connect();
  await wait(100);
  check('operator ready sees the host', opReady.peers.length === 1 && opReady.peers[0].role === 'host');
  check('host got peer-joined(operator)', hostJoined.calls.length === 1 && hostJoined.calls[0].role === 'operator');

  // --- signal relay, operator -> host --------------------------------
  op.sendSignal({ sdp: { type: 'offer', sdp: 'FAKE-OFFER' } }, hostReady.peerId);
  await wait(100);
  check('host received the operator offer verbatim',
    hostSignals.calls.length === 1 &&
    hostSignals.calls[0].from === opReady.peerId &&
    hostSignals.calls[0].data.sdp.sdp === 'FAKE-OFFER');

  // --- signal relay, host -> operator -------------------------------
  host.sendSignal({ sdp: { type: 'answer', sdp: 'FAKE-ANSWER' } }, opReady.peerId);
  host.sendSignal({ candidate: { candidate: 'FAKE-ICE' } }, opReady.peerId);
  await wait(100);
  check('operator received answer then candidate in order',
    opSignals.calls.length === 2 &&
    opSignals.calls[0].data.sdp.sdp === 'FAKE-ANSWER' &&
    opSignals.calls[1].data.candidate.candidate === 'FAKE-ICE');

  // --- list reflects the room --------------------------------------
  const lister = new SignalingClient(URL, { role: 'operator', robot: 'rover-99' });
  await lister.connect();
  const robots = await new Promise((res) => {
    lister._ws.addEventListener('message', (e) => {
      const m = JSON.parse(typeof e.data === 'string' ? e.data : e.data.toString());
      if (m.type === 'robots') res(m.robots);
    });
    lister._ws.send(JSON.stringify({ type: 'list' }));
  });
  const rover01 = robots.find((r) => r.robot === 'rover-01');
  check('list shows rover-01 online with 1 operator and a manifest',
    rover01 && rover01.online === true && rover01.operators === 1 && rover01.manifest?.robot === 'rover-01');
  lister.close();

  // --- operator leaves, host is told -----------------------------
  op.close();
  await wait(150);
  check('host got peer-left(operator)', hostLeft.calls.length === 1 && hostLeft.calls[0].role === 'operator');

  host.close();
  await wait(100);
} catch (err) {
  console.log(`FAIL  unexpected error: ${err.stack || err}`);
  failures++;
}

server.kill();
console.log(failures === 0 ? '\nall signaling smoke checks passed' : `\n${failures} check(s) failed`);
process.exit(failures === 0 ? 0 : 1);
