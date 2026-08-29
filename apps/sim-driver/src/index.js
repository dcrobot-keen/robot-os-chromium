// sim-driver — the closed-loop glue between three things that don't know
// about each other: the standalone `simulator` (a real Roboteq-protocol
// server + a ground-truth sensor stream, in its own repo on purpose --
// see simulator/README.md), pathfinder (where a path actually gets
// planned, and where the result should be visible on the real map), and
// this repo's own PathFollowerNode (roadmap.md Phase 7's last node).
//
// Everything here already runs unmodified in Node 22+ (WebSocketTransport
// uses the global WebSocket; LocalBus uses the global BroadcastChannel), so
// this is a plain Node process, not a browser page -- there's no camera or
// DOM involved, unlike pathfinder's vps-capture.html.
//
// Flow:
//   simulator :8766 (ground truth pose) --> LocalBus "<robotId>/pose"
//                                        --> PUT pathfinder /api/live-pose/<robotId>
//                                            (so it shows on the real map too)
//   pathfinder /api/drive-request/stream --> LocalBus "<robotId>/path"
//   PathFollowerNode: "<robotId>/path" + "<robotId>/pose" -> "<robotId>/drive/cmd_vel"
//   LocalBus "<robotId>/drive/cmd_vel"  --> drive.setVelocity(left, right)
//                                        --> simulator :8765 (real Roboteq frames)
//
// Run: node src/index.js
// Env: SIM_ROBOTEQ_URL, SIM_SENSOR_URL, PATHFINDER_URL, ROBOT_ID,
//      SIM_ORIGIN_X, SIM_ORIGIN_Y (where the sim's local room sits on
//      pathfinder's 200x400m plane -- default (0,0), move it if that
//      overlaps something else on the map).
import { readFile } from 'node:fs/promises';
import { WebSocketTransport, startHeartbeat, encodeCommand } from '@ros-chromium/transport';
import { createDriveDevice } from '@ros-chromium/device-abstraction';
import { LocalBus } from '@ros-chromium/bus';
import { PathFollowerNode } from '@ros-chromium/nodes';

const ROBOTEQ_URL = process.env.SIM_ROBOTEQ_URL || 'ws://127.0.0.1:8765';
const SENSOR_URL = process.env.SIM_SENSOR_URL || 'ws://127.0.0.1:8766';
const PATHFINDER_URL = process.env.PATHFINDER_URL || 'http://localhost:3001';
const ROBOT_ID = process.env.ROBOT_ID || 'tb3-sim-01';
const ORIGIN_X = Number(process.env.SIM_ORIGIN_X ?? 0);
const ORIGIN_Y = Number(process.env.SIM_ORIGIN_Y ?? 0);

const POSE_TOPIC = `${ROBOT_ID}/pose`;
const PATH_TOPIC = `${ROBOT_ID}/path`;
const CMD_TOPIC = `${ROBOT_ID}/drive/cmd_vel`;

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

// eslint-disable-next-line no-new -- wires itself up via the bus subscriptions in its constructor
new PathFollowerNode(bus, {
  pathTopic: PATH_TOPIC,
  poseTopic: POSE_TOPIC,
  cmdTopic: CMD_TOPIC,
  geometry: manifest.drive.geometry,
  onGoalReached: () => log('goal reached'),
});

// --- sensor side: ground-truth pose -> bus + pathfinder's live map ---
let lastPosePut = Promise.resolve();
function relayPoseToPathfinder(pose) {
  // Fire-and-forget, but chained so a slow PUT can't pile up faster than the
  // simulator's 10Hz scan rate produces new poses.
  lastPosePut = lastPosePut
    .catch(() => {})
    .then(() =>
      fetch(`${PATHFINDER_URL}/api/live-pose/${encodeURIComponent(ROBOT_ID)}`, {
        method: 'PUT',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ x: pose.x, y: pose.y, headingRad: pose.theta }),
      })
    )
    .catch((err) => log(`live-pose PUT to pathfinder failed: ${err.message || err}`));
}

const sensorWs = new WebSocket(SENSOR_URL);
sensorWs.addEventListener('open', () => log(`connected to simulator sensor stream ${SENSOR_URL}`));
sensorWs.addEventListener('message', (event) => {
  const msg = JSON.parse(event.data);
  if (msg.type !== 'frame') return; // the first message is {type:'hello', ...}
  const pose = { x: msg.groundTruth.x + ORIGIN_X, y: msg.groundTruth.y + ORIGIN_Y, theta: msg.groundTruth.theta };
  bus.publish(POSE_TOPIC, pose);
  relayPoseToPathfinder(pose);
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
