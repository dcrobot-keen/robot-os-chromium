// sim-driver — the closed-loop glue between three things that don't know
// about each other: the standalone `simulator` (a real Roboteq-protocol
// server + a ground-truth sensor stream, in its own repo on purpose --
// see simulator/README.md), pathfinder (where a path actually gets
// planned, and where the result should be visible on the real map), and
// this repo's own OdometryNode/PoseFusionNode/PathFollowerNode (roadmap.md
// Phase 7's remaining nodes).
//
// Everything here already runs unmodified in Node 22+ (WebSocketTransport
// uses the global WebSocket; LocalBus uses the global BroadcastChannel), so
// this is a plain Node process, not a browser page -- there's no camera or
// DOM involved, unlike pathfinder's vps-capture.html.
//
// PathFollowerNode used to steer directly off the simulator's ground-truth
// pose -- convenient for proving the drive loop worked at all, but a real
// robot never gets to see ground truth. Now it steers off PoseFusionNode's
// estimate instead, the same way a real deployment would: OdometryNode
// dead-reckons continuously from encoder counts (drifts, same as it would
// on hardware), and a throttled + noisy "VPS-like" correction (stood in
// for here by the simulator's ground truth, since there's no real vps-system
// server or camera in this loop) pulls that drift back in periodically --
// this repo's replacement for robot_localization's EKF (architecture-
// improvements.md has the fuller comparison against dc_vps_bridge).
// Ground truth itself is only used to build that stand-in correction and to
// relay a separate "-truth" debug marker to pathfinder for comparison; the
// robot never "cheats" by looking at it directly for control anymore.
//
// Flow:
//   simulator :8765 "?C" replies    --> OdometryNode --> LocalBus "<robotId>/odom"
//   simulator :8766 ground truth    --> throttled+noisy --> LocalBus "<robotId>/correction"
//                                    --> PUT pathfinder /api/live-pose/<robotId>-truth (debug only)
//   PoseFusionNode: odom + correction --> LocalBus "<robotId>/pose"
//                                      --> PUT pathfinder /api/live-pose/<robotId>
//   pathfinder /api/drive-request/stream --> LocalBus "<robotId>/path"
//   PathFollowerNode: "<robotId>/path" + "<robotId>/pose" -> "<robotId>/drive/cmd_vel"
//   LocalBus "<robotId>/drive/cmd_vel"  --> drive.setVelocity(left, right)
//                                        --> simulator :8765 (real Roboteq frames)
//
// Run: node src/index.js
// Env: SIM_ROBOTEQ_URL, SIM_SENSOR_URL, PATHFINDER_URL, ROBOT_ID,
//      SIM_ORIGIN_X, SIM_ORIGIN_Y (where the sim's local room sits on
//      pathfinder's 200x400m plane -- default (0,0), move it if that
//      overlaps something else on the map), CORRECTION_PERIOD_MS (how often
//      the VPS-like correction fires, default 2000ms to match
//      vps-capture.html's default cadence).
import { readFile } from 'node:fs/promises';
import { WebSocketTransport, startHeartbeat, encodeCommand } from '@ros-chromium/transport';
import { createDriveDevice } from '@ros-chromium/device-abstraction';
import { LocalBus } from '@ros-chromium/bus';
import { PathFollowerNode, OdometryNode, PoseFusionNode } from '@ros-chromium/nodes';

const ROBOTEQ_URL = process.env.SIM_ROBOTEQ_URL || 'ws://127.0.0.1:8765';
const SENSOR_URL = process.env.SIM_SENSOR_URL || 'ws://127.0.0.1:8766';
const PATHFINDER_URL = process.env.PATHFINDER_URL || 'http://localhost:3001';
const ROBOT_ID = process.env.ROBOT_ID || 'tb3-sim-01';
const ORIGIN_X = Number(process.env.SIM_ORIGIN_X ?? 0);
const ORIGIN_Y = Number(process.env.SIM_ORIGIN_Y ?? 0);
const CORRECTION_PERIOD_MS = Number(process.env.CORRECTION_PERIOD_MS ?? 2000);
// "VPS-like" measurement noise -- a real vps-system /localize fix has real
// error too, so the correction isn't a free perfect fix (that would make
// PoseFusionNode's job trivial and prove nothing).
const CORRECTION_NOISE_M = 0.05; // ~5cm sigma
const CORRECTION_NOISE_RAD = 0.03; // ~1.7deg sigma

const TRUTH_ID = `${ROBOT_ID}-truth`; // debug-only marker, see liveRobotPose.js
const ODOM_TOPIC = `${ROBOT_ID}/odom`;
const CORRECTION_TOPIC = `${ROBOT_ID}/correction`;
const POSE_TOPIC = `${ROBOT_ID}/pose`; // fused estimate -- what PathFollowerNode actually steers by
const PATH_TOPIC = `${ROBOT_ID}/path`;
const CMD_TOPIC = `${ROBOT_ID}/drive/cmd_vel`;

// Box-Muller gaussian, same technique as ros-chromium/simulator/src/noise.js
// (a separate, deliberately unshared copy -- see that file's own header).
function gaussianNoise(sigma) {
  let u = 0;
  let v = 0;
  while (u === 0) u = Math.random();
  while (v === 0) v = Math.random();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v) * sigma;
}

function log(msg) {
  console.log(`[sim-driver ${new Date().toISOString()}] ${msg}`);
}

const manifestUrl = new URL('../../../manifests/tb3-sim.manifest.json', import.meta.url);
const manifest = JSON.parse(await readFile(manifestUrl, 'utf-8'));

const bus = new LocalBus('sim-driver');

// --- drive side: simulator's Roboteq endpoint, same as a real dashboard ---
const transport = new WebSocketTransport(ROBOTEQ_URL);
await transport.connect();
log(`connected to simulator Roboteq endpoint ${ROBOTEQ_URL}`);
const drive = createDriveDevice(transport, manifest);
// createDriveDevice doesn't send the init sequence itself (see its header
// comment) -- roboteq-smoke.mjs is the reference for doing this by hand.
for (const line of manifest.drive.commands.init ?? []) {
  await transport.send(encodeCommand(line));
  await new Promise((r) => setTimeout(r, 30));
}
await drive.enable();
startHeartbeat(transport, {
  onGap: (gap) => log(`heartbeat gap ${gap.toFixed(0)}ms (RWD watches this)`),
  onSendError: (err) => log(`heartbeat send failed: ${err.message || err}`),
});

bus.subscribe(CMD_TOPIC, async ({ left, right }) => {
  try {
    await drive.setVelocity(left, right);
  } catch (err) {
    log(`setVelocity failed: ${err.message || err}`);
  }
});

// --- odometry + fusion: dead-reckon from encoders, correct periodically ---
// eslint-disable-next-line no-new -- polls "?C" on transport and publishes to the bus itself
new OdometryNode(bus, transport, { poseTopic: ODOM_TOPIC, geometry: manifest.drive.geometry });

// eslint-disable-next-line no-new -- wires itself up via the bus subscriptions in its constructor
new PoseFusionNode(bus, {
  odomTopic: ODOM_TOPIC,
  correctionTopic: CORRECTION_TOPIC,
  fusedTopic: POSE_TOPIC,
  // "no idea where we actually started" -- see DEFAULT_INITIAL_VARIANCE's
  // comment in pose-fusion-node.js for why this can't just be left at 0.
});

// eslint-disable-next-line no-new -- wires itself up via the bus subscriptions in its constructor
new PathFollowerNode(bus, {
  pathTopic: PATH_TOPIC,
  poseTopic: POSE_TOPIC, // the fused estimate, not ground truth -- see header comment
  cmdTopic: CMD_TOPIC,
  geometry: manifest.drive.geometry,
  onGoalReached: () => log('goal reached'),
});

// --- pathfinder relay: both the fused estimate (the real marker) and a
// "-truth" debug marker (ground truth, sim-only) so drift + correction is
// visible on the map instead of only in this process's own state ---
const posePutQueues = new Map(); // robotId -> chained Promise, so a slow PUT can't pile up
function relayPoseToPathfinder(robotId, pose) {
  const prior = posePutQueues.get(robotId) ?? Promise.resolve();
  const next = prior
    .catch(() => {})
    .then(() =>
      fetch(`${PATHFINDER_URL}/api/live-pose/${encodeURIComponent(robotId)}`, {
        method: 'PUT',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ x: pose.x, y: pose.y, headingRad: pose.theta }),
      })
    )
    .catch((err) => log(`live-pose PUT to pathfinder failed (${robotId}): ${err.message || err}`));
  posePutQueues.set(robotId, next);
}
bus.subscribe(POSE_TOPIC, (fusedPose) => relayPoseToPathfinder(ROBOT_ID, fusedPose));

// --- sensor side: ground truth -> the "VPS-like" correction + debug marker ---
// A real deployment has no ground truth at all -- vps-system's /localize
// would be the correction source instead, at whatever cadence a camera
// capture allows (pathfinder/public/vps-capture.html defaults to 2s, hence
// CORRECTION_PERIOD_MS's default). Ground truth here only stands in for
// that fix (with injected noise, so PoseFusionNode has a real job to do)
// and separately feeds the "-truth" marker so the two can be compared on
// the map.
let lastCorrectionAt = 0;
const sensorWs = new WebSocket(SENSOR_URL);
sensorWs.addEventListener('open', () => log(`connected to simulator sensor stream ${SENSOR_URL}`));
sensorWs.addEventListener('message', (event) => {
  const msg = JSON.parse(event.data);
  if (msg.type !== 'frame') return; // the first message is {type:'hello', ...}
  const truth = { x: msg.groundTruth.x + ORIGIN_X, y: msg.groundTruth.y + ORIGIN_Y, theta: msg.groundTruth.theta };
  relayPoseToPathfinder(TRUTH_ID, truth);

  const now = Date.now();
  if (now - lastCorrectionAt >= CORRECTION_PERIOD_MS) {
    lastCorrectionAt = now;
    bus.publish(CORRECTION_TOPIC, {
      x: truth.x + gaussianNoise(CORRECTION_NOISE_M),
      y: truth.y + gaussianNoise(CORRECTION_NOISE_M),
      theta: truth.theta + gaussianNoise(CORRECTION_NOISE_RAD),
    });
  }
});
sensorWs.addEventListener('error', () => log('sensor stream error'));

// --- path side: pathfinder's drive-request relay -> bus ---
function connectDriveRequestStream() {
  const wsUrl = PATHFINDER_URL.replace(/^http/, 'ws') + '/api/drive-request/stream';
  const ws = new WebSocket(wsUrl);
  ws.addEventListener('open', () => log(`subscribed to pathfinder drive-request relay ${wsUrl}`));
  ws.addEventListener('message', (event) => {
    const { robotId, path } = JSON.parse(event.data);
    if (robotId !== ROBOT_ID) return; // this relay carries every robot's requests
    // path arrives in pathfinder's plane; ORIGIN offset makes bus poses live
    // in that same plane already, so no coordinate transform is needed here.
    log(`drive request received: ${path.length} waypoints`);
    bus.publish(PATH_TOPIC, { path });
  });
  ws.addEventListener('close', () => setTimeout(connectDriveRequestStream, 2000));
  // Node's built-in WebSocket (undici) recurses into a stack overflow if
  // close() is called synchronously from within its own error dispatch --
  // unlike a browser WebSocket, which no-ops. 'close' always follows 'error'
  // here anyway and already drives the reconnect above, so just log.
  ws.addEventListener('error', (err) => log(`drive-request relay error: ${err.message || err}`));
}
connectDriveRequestStream();

log(`driving "${ROBOT_ID}" — origin offset (${ORIGIN_X}, ${ORIGIN_Y}) on pathfinder's plane`);

process.on('SIGINT', () => {
  log('shutting down');
  bus.close();
  transport.close();
  sensorWs.close();
  process.exit(0);
});
