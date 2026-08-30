// Smoke test for packages/rtc/FleetMonitor — plan.md's last step (fleet
// registry / dashboard). Proves the pass criterion directly: a robot that
// registers after the monitor is already running shows up on the very next
// poll, with no monitor-side code change needed, and disappears again once
// it disconnects (apps/signaling-server deletes an empty room outright, so
// there's no "offline" robot lingering in the list -- it's just gone).
//
// Self-contained: spawns its own signaling-server on a throwaway port.
//
//   node scripts/fleet-monitor-smoke.mjs

import { spawn } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { dirname, resolve } from 'node:path';
import { SignalingClient, FleetMonitor } from '../packages/rtc/src/signaling-client.js';

const here = dirname(fileURLToPath(import.meta.url));
const SERVER = resolve(here, '../apps/signaling-server/src/index.js');
const PORT = Number(process.env.SIGNALING_PORT || 9789);
const URL = `ws://127.0.0.1:${PORT}`;

const wait = (ms) => new Promise((r) => setTimeout(r, ms));
let failures = 0;
function check(name, cond, extra = '') {
  console.log(`${cond ? 'PASS' : 'FAIL'}  ${name}${extra ? '  (' + extra + ')' : ''}`);
  if (!cond) failures++;
}

const server = spawn(process.execPath, [SERVER], {
  env: { ...process.env, SIGNALING_PORT: String(PORT) },
  stdio: ['ignore', 'inherit', 'inherit'],
});
process.on('exit', () => server.kill());

// Collects every onUpdate() call so checks can look at the latest snapshot
// without racing the next poll.
function updateRecorder() {
  const snapshots = [];
  return { fn: (robots) => snapshots.push(robots), snapshots, latest: () => snapshots[snapshots.length - 1] };
}

try {
  await wait(400); // let the server bind

  // --- monitor starts before any robot exists -----------------------
  const monitor = new FleetMonitor(URL, { intervalMs: 50 });
  const updates = updateRecorder();
  monitor.onUpdate(updates.fn);
  monitor.connect();
  await wait(150);
  check('monitor gets an empty list before any robot registers',
    updates.snapshots.length > 0 && updates.latest().length === 0,
    `got ${JSON.stringify(updates.latest())}`);

  // --- a robot registers (with a manifest) after the monitor is already polling ---
  const host = new SignalingClient(URL, {
    role: 'host', robot: 'tb3-burger-01', manifest: { robot: 'tb3-burger-01', transport: { kind: 'turtlebot3-opencr' } },
  });
  await host.connect();
  await wait(150); // >= 3 poll intervals

  const withRobot = updates.latest();
  const tb3 = withRobot.find((r) => r.robot === 'tb3-burger-01');
  check('new robot appears on the next poll without any monitor-side code change',
    tb3 && tb3.online === true && tb3.operators === 0 && tb3.manifest?.transport?.kind === 'turtlebot3-opencr',
    `got ${JSON.stringify(tb3)}`);

  // --- a second robot, different manifest, registers too -------------
  const host2 = new SignalingClient(URL, {
    role: 'host', robot: 'former-01', manifest: { robot: 'former-01', transport: { kind: 'roboteq-serial' } },
  });
  await host2.connect();
  await wait(150);
  const withBoth = updates.latest();
  check('both robots show up side by side, each with its own manifest',
    withBoth.length === 2 &&
      withBoth.some((r) => r.robot === 'tb3-burger-01' && r.manifest?.transport?.kind === 'turtlebot3-opencr') &&
      withBoth.some((r) => r.robot === 'former-01' && r.manifest?.transport?.kind === 'roboteq-serial'),
    `got ${JSON.stringify(withBoth)}`);

  // --- an operator joining bumps the operator count on the next poll ---
  const op = new SignalingClient(URL, { role: 'operator', robot: 'former-01' });
  await op.connect();
  await wait(150);
  const formerRow = updates.latest().find((r) => r.robot === 'former-01');
  check('operator count reflects a joined operator', formerRow?.operators === 1, `got ${JSON.stringify(formerRow)}`);
  op.close();

  // --- robot disconnects -> room is deleted -> it drops off the list ---
  host.close();
  await wait(150);
  const afterDisconnect = updates.latest();
  check('a disconnected robot with no operators drops off the list entirely',
    !afterDisconnect.some((r) => r.robot === 'tb3-burger-01') &&
      afterDisconnect.some((r) => r.robot === 'former-01'),
    `got ${JSON.stringify(afterDisconnect)}`);

  // --- monitor.close() stops polling ---------------------------------
  monitor.close();
  const countAtClose = updates.snapshots.length;
  await wait(200);
  check('close() stops further polling', updates.snapshots.length === countAtClose);

  host2.close();
} catch (err) {
  console.log(`FAIL  unexpected error: ${err.stack || err}`);
  failures++;
}

server.kill();
console.log(failures === 0 ? '\nall fleet-monitor smoke checks passed' : `\n${failures} check(s) failed`);
process.exit(failures === 0 ? 0 : 1);
