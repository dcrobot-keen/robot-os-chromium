// Closed-loop check for Phase 9 step 5: spawn the simulator with the bedroom
// world (generated from a real iPhone LiDAR scan), build a likelihood field
// from the SAME iPhone-scan slicemap, and verify LocalizationNode ->
// PoseFusionNode keeps the estimate locked to the sim's ground truth while
// driving with sensor noise on -- i.e. an iPhone map used on the robot, in
// reality (roadmap.md Phase 9's whole point).
//
// Spawns ../../simulator, like roboteq-smoke.mjs spawns ../../robot-base/sim.
//
//   node scripts/localization-sim-smoke.mjs
import { spawn } from 'node:child_process';
import { readFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { dirname, resolve } from 'node:path';
import { WebSocketTransport, startHeartbeat } from '@ros-chromium/transport';
import { createDriveDevice } from '@ros-chromium/device-abstraction';
import { LocalBus } from '@ros-chromium/bus';
import {
  OdometryNode, PoseFusionNode, LocalizationNode,
  buildLikelihoodField, parseSlicemap, wallMaskFromSlicemap,
} from '@ros-chromium/nodes';

const here = dirname(fileURLToPath(import.meta.url));
const SIM = resolve(here, '../../simulator/src/index.js');
const SLICEMAP = resolve(here, '../../simulator/worlds/bedroom.slicemap.json');
const WORLD = 'worlds/bedroom.world.json';
const RPORT = 8801, SPORT = 8802, VPORT = 8803;

const wait = (ms) => new Promise((r) => setTimeout(r, ms));
let failures = 0;
const check = (name, cond, extra = '') => {
  console.log(`${cond ? 'PASS' : 'FAIL'}  ${name}${extra ? '  (' + extra + ')' : ''}`);
  if (!cond) failures++;
};

const manifest = JSON.parse(await readFile(new URL('../manifests/tb3-sim.manifest.json', import.meta.url)));
const slice = parseSlicemap(JSON.parse(await readFile(SLICEMAP, 'utf-8')));
// slicemap-to-world.mjs places the walls at slice-local coords (origin 0),
// and the sim's ground truth is in that frame -- so the field must be too.
const grid = { originX: 0, originY: 0, cellSize: slice.resolution, cols: slice.cols, rows: slice.rows };
const lf = buildLikelihoodField(grid, wallMaskFromSlicemap(slice), { sigmaM: 0.08 });
check('likelihood field built from the bedroom slicemap', lf.field.some((v) => v > 0.5));

const sim = spawn(process.execPath, [SIM], {
  env: {
    ...process.env,
    SIM_PORT: String(RPORT), SIM_SENSOR_PORT: String(SPORT), SIM_VIEWER_PORT: String(VPORT),
    SIM_WORLD: WORLD, SIM_NOISE: 'default', SIM_SEED: '4',
  },
  stdio: ['ignore', 'ignore', 'inherit'],
});
process.on('exit', () => sim.kill());

try {
  await wait(600);

  const transport = new WebSocketTransport(`ws://127.0.0.1:${RPORT}`);
  await transport.connect();
  const drive = createDriveDevice(transport, manifest);
  for (const line of manifest.drive.commands.init) { await transport.send(transport.encode(line)); await wait(20); }
  await drive.enable();
  startHeartbeat(transport);

  const bus = new LocalBus('localization-sim-smoke');
  const geometry = manifest.drive.geometry;

  // eslint-disable-next-line no-new
  new OdometryNode(bus, transport, { poseTopic: 'odom', geometry, encoderQuery: manifest.drive.readback.encoder });
  // eslint-disable-next-line no-new
  new PoseFusionNode(bus, { odomTopic: 'odom', correctionTopic: 'correction', fusedTopic: 'pose' });

  let groundTruth = null;
  let firstGT = null;
  const errs = [];
  bus.subscribe('pose', (p) => { if (groundTruth) errs.push(Math.hypot(p.x - groundTruth.x, p.y - groundTruth.y)); });

  const sensor = new WebSocket(`ws://127.0.0.1:${SPORT}`);
  sensor.binaryType = 'arraybuffer';
  let loc = null;
  sensor.addEventListener('message', (ev) => {
    const msg = JSON.parse(typeof ev.data === 'string' ? ev.data : new TextDecoder().decode(ev.data));
    if (msg.type !== 'frame') return;
    groundTruth = msg.groundTruth;
    if (!firstGT) {
      firstGT = { ...msg.groundTruth };
      loc = new LocalizationNode(bus, {
        scanTopic: 'scan', odomTopic: 'odom', correctionTopic: 'correction',
        likelihoodField: lf, initialPose: firstGT, minScore: 0.3, lostAfter: 8,
      });
    }
    bus.publish('scan', msg.scan);
  });

  await wait(400); // let the first frame + LocalizationNode come up

  // drive a path: forward, arc, forward. Moderate speed -- the walls-only
  // bedroom world is open enough; localization only needs motion + scans.
  const send = (l, r) => drive.setVelocity(l, r);
  for (let i = 0; i < 45; i++) { await send(0.5, 0.5); await wait(50); }
  for (let i = 0; i < 30; i++) { await send(0.35, 0.6); await wait(50); }
  for (let i = 0; i < 45; i++) { await send(0.5, 0.5); await wait(50); }
  await send(0, 0);
  await wait(300);

  transport.close();
  sensor.close();

  const settled = errs.slice(Math.floor(errs.length / 3)); // drop the initial convergence
  const meanErr = settled.reduce((a, b) => a + b, 0) / settled.length;
  const maxErr = Math.max(...settled);
  const finalErr = errs.at(-1);
  const movedGT = Math.hypot(groundTruth.x - firstGT.x, groundTruth.y - firstGT.y);

  check('robot actually moved in the bedroom world', movedGT > 0.4, `${movedGT.toFixed(2)} m from start`);
  check('LocalizationNode relocated the fused pose onto the map', errs.length > 30, `${errs.length} pose samples`);
  check('fused pose stays locked to ground truth over the drive (noise on)',
    meanErr < 0.15 && maxErr < 0.35,
    `mean ${(meanErr * 100).toFixed(1)} cm, max ${(maxErr * 100).toFixed(1)} cm, final ${(finalErr * 100).toFixed(1)} cm`);

  loc?.stop();
  bus.close();
} catch (err) {
  console.log(`FAIL  unexpected error: ${err.stack || err}`);
  failures++;
}

sim.kill();
console.log(failures === 0 ? '\nall localization-sim smoke checks passed' : `\n${failures} check(s) failed`);
process.exit(failures === 0 ? 0 : 1);
