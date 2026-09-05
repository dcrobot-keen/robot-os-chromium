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
// With MQTT_URL set, Vda5050Node (packages/nodes) additionally speaks
// VDA5050 to a broker -- doc/vda5050-rcs.md in the workspace root:
//   LocalBus "<robotId>/pose"        --> MQTT uagv/v2/<manufacturer>/<robotId>/visualization + /state
//   MQTT .../order                   --> LocalBus "<robotId>/path"   (same topic the HTTP relay feeds)
//   MQTT .../instantActions          --> cancelOrder / stopPause / startPause
// In that mode the fused pose is NOT also PUT to pathfinder's live-pose (the
// RCS gets it over MQTT); the "-truth" debug marker still goes over HTTP.
//
// Run: node src/index.js
// Env: SIM_ROBOTEQ_URL, SIM_SENSOR_URL, PATHFINDER_URL, ROBOT_ID,
//      GOAL_TOLERANCE_M (PathFollowerNode goal radius, default 0.2),
//      MAX_DEVIATION_M (safety stop when the fused pose leaves the path, default 0.8; 0 = off),
//      SIM_SLICEMAP (slicemap the world came from -> static layer; e.g. the SIM_WORLD value),
//      STATIC_MARGIN_M (dilation of the static layer before subtracting, default 0.5),
//      DYNAMIC_PERSIST (map updates a dynamic cell must persist before it blocks, default 8),
//      BLOCK_CHECK (dynamic (default with SIM_SLICEMAP) | occupied | inflated | off),
//      BATTERY_POLL_MS (Roboteq "?V 2" poll for VDA5050 batteryState, default 5000; 0 = off),
//      MQTT_URL (e.g. mqtt://mosquitto:1883; unset = no VDA5050), VDA_MANUFACTURER
//      (default dcrobot), MAP_ID (VDA5050 mapId = pathfinder project, default
//      "default"), VDA_NODE_REACHED_M (node-reached radius, default 0.35),
//      SIM_ORIGIN_X, SIM_ORIGIN_Y (where the sim's local room sits on
//      pathfinder's 200x400m plane -- default (0,0), move it if that
//      overlaps something else on the map), CORRECTION_PERIOD_MS (how often
//      the VPS-like correction fires, default 2000ms to match
//      vps-capture.html's default cadence).
import { readFile } from 'node:fs/promises';
import { WebSocketTransport, startHeartbeat } from '@ros-chromium/transport';
import { createDriveDevice } from '@ros-chromium/device-abstraction';
import { LocalBus } from '@ros-chromium/bus';
import { PathFollowerNode, OdometryNode, PoseFusionNode, MapNode, CostmapNode, gridConfigFromBounds, parseSlicemap, inflateOccupancy, Vda5050Node, adaptMqttJsClient } from '@ros-chromium/nodes';
import { existsSync } from 'node:fs';

const ROBOTEQ_URL = process.env.SIM_ROBOTEQ_URL || 'ws://127.0.0.1:8765';
const SENSOR_URL = process.env.SIM_SENSOR_URL || 'ws://127.0.0.1:8766';
const PATHFINDER_URL = process.env.PATHFINDER_URL || 'http://localhost:3001';
const ROBOT_ID = process.env.ROBOT_ID || 'tb3-sim-01';
const ORIGIN_X = Number(process.env.SIM_ORIGIN_X ?? 0);
const ORIGIN_Y = Number(process.env.SIM_ORIGIN_Y ?? 0);
const CORRECTION_PERIOD_MS = Number(process.env.CORRECTION_PERIOD_MS ?? 2000);
const MQTT_URL = process.env.MQTT_URL || '';
const VDA_MANUFACTURER = process.env.VDA_MANUFACTURER || 'dcrobot';
// VDA5050 mapId. Unset -> taken from the simulator's world name (its hello
// message), which for a published slicemap equals the pathfinder project made
// from the same file (pathfinder POST /api/projects/from-slicemap).
const MAP_ID = process.env.MAP_ID || '';
const VDA_NODE_REACHED_M = Number(process.env.VDA_NODE_REACHED_M ?? 0.35);
const MAX_DEVIATION_M = Number(process.env.MAX_DEVIATION_M ?? 0.8);
// Dynamic-obstacle stop. Checking pathfinder-planned paths against the raw/inflated LIDAR
// map aborted every wall-side drive (the path keeps 0.15-0.2 m from slicemap walls; sensor
// noise + 0.2-0.4 m fused-pose error land those cells on it). The trustworthy version is
// the 'dynamic' layer: CostmapNode subtracts a static mask built from the SAME slicemap the
// world (and pathfinder's plan) came from, dilated by STATIC_MARGIN_M so wobbling wall hits
// don't count, and only what is left -- a new box, a person -- stops the robot.
// SIM_SLICEMAP (path under /app/simulator, e.g. the SIM_WORLD value) enables it.
const SIM_SLICEMAP = process.env.SIM_SLICEMAP || '';
const STATIC_MARGIN_M = Number(process.env.STATIC_MARGIN_M ?? 0.5);
// consecutive MapNode updates a cell must stay dynamic before it can stop the robot: filters
// the wall hits a heading jump paints 0.4-0.6 m off the wall for a few frames (seen 2026-09-05).
const DYNAMIC_PERSIST = Number(process.env.DYNAMIC_PERSIST ?? 8);
const BLOCK_CHECK = (process.env.BLOCK_CHECK || (SIM_SLICEMAP ? 'dynamic' : 'off')).toLowerCase(); // dynamic | occupied | inflated | off
const COSTMAP_TOPIC = `${ROBOT_ID}/costmap`;
const BATTERY_POLL_MS = Number(process.env.BATTERY_POLL_MS ?? 5000);
// TB3 Burger 3-cell LiPo: 12.6 V full, ~11.0 V "go home". The simulator answers
// "?V 2" with V=<volts*10> (a fixed 12.0 V today), so this mostly proves the plumbing.
const BATTERY_FULL_V = 12.6, BATTERY_EMPTY_V = 11.0;
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
const SCAN_TOPIC = `${ROBOT_ID}/scan`; // sim sensor stream's laser scan, put on the bus for MapNode
const MAP_TOPIC = `${ROBOT_ID}/map`; // MapNode's occupancy grid (raw + body-inflated)

// MapNode grid: a fixed window on pathfinder's plane, centred on the sim's
// local room origin. The sim worlds top out around 14x10m; MAP_SPAN gives
// margin on every side. cellSize 0.05m matches the LDS-01's angular
// resolution at close range without exploding the cell count (SPAN/cs)^2.
const MAP_SPAN = Number(process.env.MAP_SPAN ?? 20); // metres, square, half each side of origin
const MAP_CELL = Number(process.env.MAP_CELL ?? 0.05);
// TB3 Burger body radius ~0.11m; a bit more keeps pure pursuit off the walls.
const MAP_INFLATION = Number(process.env.MAP_INFLATION ?? 0.18);

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
  await transport.send(transport.encode(line));
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
// eslint-disable-next-line no-new -- polls the encoder query on transport and publishes to the bus itself
new OdometryNode(bus, transport, {
  poseTopic: ODOM_TOPIC,
  geometry: manifest.drive.geometry,
  encoderQuery: manifest.drive.readback?.encoder, // "?C" for Roboteq; undefined -> node default
});

// eslint-disable-next-line no-new -- wires itself up via the bus subscriptions in its constructor
new PoseFusionNode(bus, {
  odomTopic: ODOM_TOPIC,
  correctionTopic: CORRECTION_TOPIC,
  fusedTopic: POSE_TOPIC,
  // "no idea where we actually started" -- see DEFAULT_INITIAL_VARIANCE's
  // comment in pose-fusion-node.js for why this can't just be left at 0.
});

// --- mapping: laser scans + fused pose -> occupancy grid on the bus ---
// Poses on POSE_TOPIC already live in pathfinder's plane (ORIGIN offset
// applied below), so the grid is defined there too and MapNode's output
// drops straight into a PlannerNode request.
// With a slicemap world the grid IS the slicemap grid placed like the simulator places it:
// grid corner at (ORIGIN_X, ORIGIN_Y) -- see simulator/src/slicemap.js toWorld and pathfinder's
// server/slicemap.mjs, which drop the slicemap's own origin the same way.
let staticSlice = null;
if (SIM_SLICEMAP) {
  const p = SIM_SLICEMAP.startsWith('/') ? SIM_SLICEMAP : new URL(`../../../../simulator/${SIM_SLICEMAP}`, import.meta.url).pathname;
  const path = decodeURIComponent(p);
  if (!existsSync(path)) log(`SIM_SLICEMAP ${path} not found -- dynamic block check off`);
  else {
    try {
      staticSlice = parseSlicemap(JSON.parse(await readFile(path, 'utf-8')));
      log(`static layer from slicemap ${path} (${staticSlice.cols}x${staticSlice.rows} @ ${staticSlice.resolution} m)`);
    } catch (err) {
      log(`SIM_SLICEMAP unreadable (${err.message}) -- dynamic block check off`);
    }
  }
}
const mapGrid = staticSlice
  ? { originX: ORIGIN_X, originY: ORIGIN_Y, cellSize: staticSlice.resolution, cols: staticSlice.cols, rows: staticSlice.rows }
  : gridConfigFromBounds(
      { minX: ORIGIN_X - MAP_SPAN, minY: ORIGIN_Y - MAP_SPAN, maxX: ORIGIN_X + MAP_SPAN, maxY: ORIGIN_Y + MAP_SPAN },
      MAP_CELL
    );
let lastMapLogAt = 0;
// eslint-disable-next-line no-new -- wires itself up via the bus subscriptions in its constructor
new MapNode(bus, {
  scanTopic: SCAN_TOPIC,
  poseTopic: POSE_TOPIC,
  mapTopic: MAP_TOPIC,
  grid: mapGrid,
  inflationRadius: MAP_INFLATION,
  onUpdate: (map) => {
    if (Date.now() - lastMapLogAt < 5000) return;
    lastMapLogAt = Date.now();
    const occ = map.occupied.reduce((n, c) => n + (c ? 1 : 0), 0);
    const inf = map.occupiedInflated.reduce((n, c) => n + (c ? 1 : 0), 0);
    log(`map: ${occ} occupied / ${inf} inflated of ${map.cols}x${map.rows} cells`);
  },
});
log(`MapNode grid ${mapGrid.cols}x${mapGrid.rows} @ ${mapGrid.cellSize}m, origin (${mapGrid.originX.toFixed(2)}, ${mapGrid.originY.toFixed(2)}), inflation ${MAP_INFLATION}m`);

// --- costmap: static (slicemap walls+furniture, dilated) vs dynamic (everything else) ---
let costmapNode = null;
if (staticSlice && BLOCK_CHECK !== 'off') {
  const staticRaw = Array.from(staticSlice.codes, (c) => c === 2 || c === 3);
  // The simulator's world boundary (bounds rectangle) is a wall for the LIDAR but not a
  // slicemap cell -- add the outer ring so boundary hits don't read as dynamic obstacles.
  for (let c = 0; c < mapGrid.cols; c++) { staticRaw[c] = true; staticRaw[(mapGrid.rows - 1) * mapGrid.cols + c] = true; }
  for (let r = 0; r < mapGrid.rows; r++) { staticRaw[r * mapGrid.cols] = true; staticRaw[r * mapGrid.cols + mapGrid.cols - 1] = true; }
  const staticMask = inflateOccupancy(staticRaw, mapGrid, STATIC_MARGIN_M);
  let lastCostmapLog = 0;
  costmapNode = new CostmapNode(bus, {
    mapTopic: MAP_TOPIC, costmapTopic: COSTMAP_TOPIC, poseTopic: POSE_TOPIC,
    staticOccupied: staticMask, grid: mapGrid,
    inflationRadius: MAP_INFLATION, dynamicInflationRadius: MAP_INFLATION, dynamicPersistUpdates: DYNAMIC_PERSIST, dynThreshold: 0.6,
    onUpdate: (cm) => {
      // log every 5 s, or every 0.5 s while something dynamic is visible (debug the transients)
      if (Date.now() - lastCostmapLog < (cm.dynamicCount > 0 ? 500 : 5000)) return;
      lastCostmapLog = Date.now();
      const sample = [];
      for (let i = 0; i < cm.dynamic.length && sample.length < 3; i++) {
        if (cm.dynamic[i]) sample.push(`(${(cm.originX + ((i % cm.cols) + 0.5) * cm.cellSize).toFixed(2)}, ${(cm.originY + (Math.floor(i / cm.cols) + 0.5) * cm.cellSize).toFixed(2)})`);
      }
      log(`costmap: ${cm.dynamicCount} dynamic cells (${cm.dynamicPersistentCount} persistent >${DYNAMIC_PERSIST} updates) beyond the static layer (margin ${STATIC_MARGIN_M} m)${sample.length ? ' e.g. ' + sample.join(' ') : ''}`);
    },
  });
  log(`CostmapNode: static mask ${staticMask.reduce((n, v) => n + (v ? 1 : 0), 0)} cells, block check layer '${BLOCK_CHECK}'`);
}

// eslint-disable-next-line no-new -- wires itself up via the bus subscriptions in its constructor
new PathFollowerNode(bus, {
  pathTopic: PATH_TOPIC,
  poseTopic: POSE_TOPIC, // the fused estimate, not ground truth -- see header comment
  cmdTopic: CMD_TOPIC,
  geometry: manifest.drive.geometry,
  // 0.1 m (node default) is tighter than the fused estimate's error between
  // corrections (~0.2 m at 2 s / 0.12 m/s), so the robot used to orbit the goal.
  goalToleranceM: Number(process.env.GOAL_TOLERANCE_M ?? 0.2),
  maxDeviationM: MAX_DEVIATION_M,
  // Dynamic-obstacle stop (see BLOCK_CHECK above): 'dynamic' reads CostmapNode's
  // dynamicInflated; 'occupied'/'inflated' fall back to MapNode's raw grid (experiments).
  costmapTopic: BLOCK_CHECK === 'off' ? undefined : BLOCK_CHECK === 'dynamic' ? (costmapNode ? COSTMAP_TOPIC : undefined) : MAP_TOPIC,
  blockedLayer: BLOCK_CHECK,
  onBlocked: ({ pose, blockedIndex, blockedPoint }) => {
    const at = blockedPoint ? `(${blockedPoint[0].toFixed(2)}, ${blockedPoint[1].toFixed(2)})` : '?';
    log(`SAFETY STOP: path blocked by dynamic obstacle at waypoint ${blockedIndex} ${at}, robot at (${pose.x.toFixed(2)}, ${pose.y.toFixed(2)}, ${pose.theta.toFixed(2)})`);
    vda?.abortOrder('obstacleBlocked', `path blocked by obstacle at waypoint ${blockedIndex} ${at}`);
  },
  onGoalReached: () => {
    log('goal reached');
    vda?.completeOrder(); // VDA5050: close out any nodes the radius test skipped
  },
  onAbort: ({ reason, distance, limit }) => {
    log(`SAFETY STOP: ${reason} (${distance.toFixed(2)} m off the path, limit ${limit} m)`);
    vda?.abortOrder(reason, `fused pose ${distance.toFixed(2)} m from the path (limit ${limit} m)`);
  },
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
if (!MQTT_URL) bus.subscribe(POSE_TOPIC, (fusedPose) => relayPoseToPathfinder(ROBOT_ID, fusedPose));

// --- VDA5050 over MQTT (optional): the standard replacement for the two
// HTTP relays above/below. Vda5050Node only sees { publish, subscribe }; the
// `mqtt` package client is wrapped by adaptMqttJsClient. The client's last
// will publishes CONNECTIONBROKEN on the connection topic if this process
// dies without close().
// --- battery: poll the Roboteq "?V 2" readback (manifest.drive.readback.battery) ---
// Replies arrive on the same transport as OdometryNode's "?C" answers; we only
// look at V= lines. Exposed to Vda5050Node as batteryState.
let batteryVolts = null;
if (BATTERY_POLL_MS > 0 && manifest.drive.readback?.battery) {
  transport.onMessage((m) => {
    if (m.type === 'reply' && m.key === 'V' && Array.isArray(m.values) && m.values.length) batteryVolts = m.values[m.values.length - 1] / 10;
  });
  setInterval(() => transport.send(transport.encode(manifest.drive.readback.battery)).catch(() => {}), BATTERY_POLL_MS);
}
const batteryState = () => {
  if (batteryVolts == null) return undefined;
  const pct = Math.max(0, Math.min(100, ((batteryVolts - BATTERY_EMPTY_V) / (BATTERY_FULL_V - BATTERY_EMPTY_V)) * 100));
  return { batteryCharge: Math.round(pct), batteryVoltage: batteryVolts, charging: false };
};

let vda = null;
if (MQTT_URL) {
  const { default: mqttLib } = await import('mqtt');
  const ids = { manufacturer: VDA_MANUFACTURER, serialNumber: ROBOT_ID };
  const client = mqttLib.connect(MQTT_URL, {
    clientId: `vda5050-${ROBOT_ID}-${Math.random().toString(16).slice(2, 8)}`,
    will: Vda5050Node.lastWill(ids),
    reconnectPeriod: 2000,
  });
  client.on('connect', () => log(`MQTT connected ${MQTT_URL}`));
  client.on('reconnect', () => log('MQTT reconnecting'));
  client.on('error', (err) => log(`MQTT error: ${err.message || err}`));
  await new Promise((resolve) => client.once('connect', resolve));
  vda = new Vda5050Node(bus, adaptMqttJsClient(client), {
    ...ids,
    mapId: MAP_ID || 'default',
    poseTopic: POSE_TOPIC,
    pathTopic: PATH_TOPIC,
    cmdTopic: CMD_TOPIC,
    nodeReachedM: VDA_NODE_REACHED_M,
    battery: BATTERY_POLL_MS > 0 ? () => batteryState() : null,
    log: (m) => log(`vda5050: ${m}`),
  });
  process.on('SIGINT', () => {
    vda.close();
    client.end();
  });
}

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
  if (msg.type === 'hello' && !MAP_ID && msg.world?.name) vda?.setMapId(msg.world.name);
  if (msg.type !== 'frame') return; // the first message is {type:'hello', ...}

  // laser scan -> bus, for MapNode. ranges/angles are robot-relative, so no
  // ORIGIN offset here -- MapNode places them using the fused pose, which is
  // already in pathfinder's plane.
  if (msg.scan) bus.publish(SCAN_TOPIC, msg.scan);

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
