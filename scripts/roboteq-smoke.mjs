// Smoke test for the Roboteq line protocol end to end, no browser: the
// WebSocketTransport + roboteq.js codec + the firmware sim's emulator +
// createDriveDevice, plus the load-bearing RWD watchdog.
//
// Self-contained: spawns the firmware sim on a throwaway port with a short
// SIM_RWD_MS, runs the checks, prints PASS/FAIL, exits non-zero on failure.
//
//   node scripts/roboteq-smoke.mjs

import { spawn } from 'node:child_process';
import { readFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { dirname, resolve } from 'node:path';
import { WebSocketTransport } from '../packages/transport/src/index.js';
import { createDriveDevice } from '../packages/device-abstraction/src/drive-device.js';

const here = dirname(fileURLToPath(import.meta.url));
const SIM = resolve(here, '../../firmware/sim/src/index.js');
const manifest = JSON.parse(await readFile(new URL('../manifests/former.manifest.json', import.meta.url)));
const PORT = 8791;
const RWD_MS = 400;

const wait = (ms) => new Promise((r) => setTimeout(r, ms));
let failures = 0;
function check(name, cond) {
  console.log(`${cond ? 'PASS' : 'FAIL'}  ${name}`);
  if (!cond) failures++;
}

let simLog = '';
const sim = spawn(process.execPath, [SIM], {
  env: { ...process.env, SIM_PORT: String(PORT), SIM_RWD_MS: String(RWD_MS) },
  stdio: ['ignore', 'pipe', 'inherit'],
});
sim.stdout.on('data', (d) => { simLog += d.toString(); process.stdout.write(d); });
process.on('exit', () => sim.kill());

try {
  await wait(400);

  const transport = new WebSocketTransport(`ws://127.0.0.1:${PORT}`);
  const replies = [];
  transport.onMessage((m) => replies.push(m));
  await transport.connect();

  const drive = createDriveDevice(transport, manifest);
  const enc = new TextEncoder();

  // --- ?FID handshake --------------------------------------------------
  await transport.send(enc.encode('?FID\r'));
  await wait(50);
  check('?FID returns a FID= line', replies.some((m) => m.type === 'reply' && m.key === 'FID'));

  // --- manifest init sequence is all accepted -----------------------
  const initCmds = manifest.drive.commands.init;
  const expectedAcks = initCmds.reduce((n, line) => n + line.split('_').length, 0);
  const ackBefore = replies.filter((m) => m.type === 'ack' && m.ok).length;
  for (const line of initCmds) { await transport.send(enc.encode(line + '\r')); await wait(30); }
  await wait(50);
  const ackAfter = replies.filter((m) => m.type === 'ack' && m.ok).length;
  check('every init sub-command is "+"-acked', ackAfter - ackBefore === expectedAcks);

  // --- !G ignored until !MG ------------------------------------------
  await drive.setVelocity(0.5, 0.5);
  await wait(50);
  await transport.send(new TextEncoder().encode('?C\r'));
  await wait(50);
  let c = replies.filter((m) => m.type === 'reply' && m.key === 'C').at(-1);
  check('encoders idle before !MG', c && c.values[0] === 0 && c.values[1] === 0);

  // --- enable + drive -> encoders advance --------------------------
  await drive.enable();
  await drive.setVelocity(0.5, 0.5);
  await wait(250);
  await transport.send(new TextEncoder().encode('?C\r'));
  await wait(50);
  c = replies.filter((m) => m.type === 'reply' && m.key === 'C').at(-1);
  check('encoders advance while driving', c && c.values[0] > 0 && c.values[1] > 0);

  // --- ack on a command ---------------------------------------------
  check('commands are acked with "+"', replies.some((m) => m.type === 'ack' && m.ok));

  // --- !EX stops and latches --------------------------------------
  await drive.estop();
  await wait(50);
  await transport.send(new TextEncoder().encode('?FF_?DI\r'));
  await wait(50);
  const ff = replies.filter((m) => m.type === 'reply' && m.key === 'FF').at(-1);
  const di = replies.filter((m) => m.type === 'reply' && m.key === 'DI').at(-1);
  check('after !EX: FF=16 (motor disabled)', ff && ff.values[0] === 16);
  check('after !EX: DI shows estopped (0)', di && di.values[0] === 0);

  // --- RWD watchdog fires on silence -----------------------------
  // Re-enable, drive, then go quiet and let the sim's RWD stop it.
  await drive.enable();
  await drive.setVelocity(0.4, 0.4);
  const before = simLog.length;
  await wait(RWD_MS + 300);
  check('RWD watchdog stops motors after silence',
    simLog.slice(before).includes('RWD: no serial command'));

  transport.close();
  await wait(50);
} catch (err) {
  console.log(`FAIL  unexpected error: ${err.stack || err}`);
  failures++;
}

sim.kill();
console.log(failures === 0 ? '\nall roboteq smoke checks passed' : `\n${failures} check(s) failed`);
process.exit(failures === 0 ? 0 : 1);
