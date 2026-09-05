// Smoke for the real-robot safety/localization pieces added 2026-09-05:
//   1. vps-correction-node.js: ARKit -> ground pose, frame (ScanAlignment) application,
//      /localize client (fake fetch), VpsCorrectionNode capture->localize->correction loop
//   2. PathFollowerNode.maxDeviationM -> onAbort + zero velocity
//   3. Vda5050Node.abortOrder / estopTopic -> FATAL error / safetyState
// No camera, no server, no broker: everything injected.
//
//   node scripts/vps-correction-smoke.mjs
import { LocalBus } from '@ros-chromium/bus';
import {
  VpsCorrectionNode, arkitPoseToGroundPose, applyFrame, vpsResultToPose, createVpsLocalizeClient,
  PathFollowerNode, distanceToPath, Vda5050Node, pathToOrder,
} from '@ros-chromium/nodes';

let failures = 0;
function check(name, cond, extra = '') {
  console.log(`${cond ? 'PASS' : 'FAIL'}  ${name}${extra ? '  (' + extra + ')' : ''}`);
  if (!cond) failures++;
}
const near = (a, b, tol = 1e-6) => Math.abs(a - b) < tol;
const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));

// --- 1a. ARKit -> ground pose ---
{
  // identity rotation: camera forward = -Z (ARKit) -> slice-plane +y -> theta = +90deg
  const p = arkitPoseToGroundPose([1, 1.4, -2], [0, 0, 0, 1]);
  check('identity camera at (1, h, -2) -> ground (1, 2) facing +y', near(p.x, 1) && near(p.y, 2) && near(p.theta, Math.PI / 2));
  // yaw 90deg about Y (q = [0, sin45, 0, cos45]): forward -Z rotates to -X -> theta = 180deg
  const s = Math.SQRT1_2;
  const q = arkitPoseToGroundPose([0, 0, 0], [0, s, 0, s]);
  check('90deg yaw about Y -> heading pi', near(Math.abs(wrap(q.theta)), Math.PI, 1e-6));
}

// --- 1b. frame (ScanAlignment) application matches the canonical vectors ---
{
  const frame = { offsetX: 1.5, offsetZ: -0.5, yawRadians: Math.PI / 6 };
  const pose = { x: 0.7, y: 0.2, theta: 0 }; // slice plane, ARKit (0.7, -0.2)
  const out = applyFrame(pose, frame);
  const c = Math.cos(frame.yawRadians), sn = Math.sin(frame.yawRadians);
  const ex = 0.7 * c + -0.2 * sn + 1.5, ez = -0.7 * sn + -0.2 * c - 0.5;
  check('applyFrame position = ARKit applyXZ then y=-z', near(out.x, ex) && near(out.y, -ez));
  // heading: slice-plane rotation by +yaw (CCW)
  check('applyFrame rotates heading by +yaw', near(wrap(out.theta - (pose.theta + frame.yawRadians)), 0, 1e-6), out.theta.toFixed(4));
  check('applyFrame(null) is identity', applyFrame(pose, null) === pose);
}

// --- 1c. result -> pose, client ---
{
  const res = { room_id: 'scan_A', translation: [2, 1.3, -3], quaternion: [0, 0, 0, 1], num_inliers: 40, frame: { mapId: 'project_x', alignment: { offsetX: 0, offsetZ: 0, yawRadians: 0 } } };
  const pose = vpsResultToPose(res);
  check('vpsResultToPose carries room/map ids', pose.x === 2 && pose.y === 3 && pose.roomId === 'scan_A' && pose.mapId === 'project_x' && pose.inliers === 40);
  check('few inliers -> null', vpsResultToPose({ ...res, num_inliers: 5 }) === null && vpsResultToPose({}) === null);

  const calls = [];
  const fakeFetch = async (url, init) => {
    calls.push({ url, form: init.body });
    return { ok: true, status: 200, json: async () => res };
  };
  const localize = createVpsLocalizeClient('http://vps.test:8000/', { fx: 500, fy: 500, cx: 320, cy: 240, width: 640, height: 480 }, { fetchImpl: fakeFetch });
  const out = await localize(new Blob(['jpeg'], { type: 'image/jpeg' }));
  check('client posts multipart to /localize with intrinsics', calls[0].url === 'http://vps.test:8000/localize' && calls[0].form.get('fx') === '500' && calls[0].form.get('image') instanceof Blob && out.room_id === 'scan_A');
  const failing = createVpsLocalizeClient('http://vps.test:8000', { fx: 1, fy: 1, cx: 1, cy: 1, width: 1, height: 1 }, { fetchImpl: async () => ({ ok: false, status: 422, json: async () => ({ detail: 'no fix' }) }) });
  let err = null; try { await failing(new Blob(['x'])); } catch (e) { err = e; }
  check('422 surfaces as an error with detail + status', err?.status === 422 && err.message === 'no fix');
  let threw = false; try { createVpsLocalizeClient('http://x', { fx: 'a' }); } catch { threw = true; } check('bad intrinsics rejected early', threw);
}

// --- 1d. VpsCorrectionNode loop with manual timers ---
{
  const bus = new LocalBus('vps-smoke');
  const corrections = [];
  bus.subscribe('r/correction', (c) => corrections.push(c));
  const timers = [];
  let localizeResult = { room_id: 'scan_A', translation: [1, 1, -1], quaternion: [0, 0, 0, 1], num_inliers: 30 };
  let captureCount = 0;
  const misses = [];
  const node = new VpsCorrectionNode(bus, {
    capture: async () => { captureCount++; return new Blob(['img']); },
    localize: async () => { if (localizeResult instanceof Error) throw localizeResult; return localizeResult; },
    correctionTopic: 'r/correction', periodMs: 1000,
    onMiss: (e) => misses.push(e.message),
    setTimer: (fn, ms) => { timers.push({ fn, ms }); return timers.length; },
    clearTimer: () => {},
  });
  node.start();
  check('start schedules an immediate tick', timers.length === 1 && timers[0].ms === 0);
  await timers[0].fn();
  check('tick: capture -> localize -> correction published', captureCount === 1 && corrections.length === 1 && corrections[0].x === 1 && corrections[0].y === 1 && near(corrections[0].theta, Math.PI / 2));
  check('next tick scheduled after periodMs', timers.at(-1).ms === 1000 && node.stats.fixes === 1);
  localizeResult = Object.assign(new Error('no fix'), { status: 422 });
  await node.tick();
  check('a miss publishes nothing and keeps the loop alive', corrections.length === 1 && node.stats.misses === 1 && misses[0] === 'no fix' && timers.at(-1).ms === 1000);
  localizeResult = { room_id: 'scan_A', translation: [0, 0, 0], quaternion: [0, 0, 0, 1], num_inliers: 3 };
  await node.tick();
  check('low-inlier answer counts as a miss', corrections.length === 1 && node.stats.misses === 2);
  node.stop();
  const before = timers.length;
  await node.tick();
  check('after stop() a tick does not reschedule', timers.length === before);
  bus.close();
}

// --- 2. PathFollowerNode deviation abort ---
{
  check('distanceToPath: perpendicular to a segment', near(distanceToPath([[0, 0], [2, 0]], { x: 1, y: 0.5 }), 0.5));
  check('distanceToPath: beyond the end clamps to the endpoint', near(distanceToPath([[0, 0], [2, 0]], { x: 3, y: 0 }), 1));
  const bus = new LocalBus('dev-smoke');
  const cmds = [];
  const aborts = [];
  bus.subscribe('r/cmd', (c) => cmds.push(c));
  const geometry = { wheelRadius: 0.033, wheelSeparation: 0.16, maxWheelRpm: 63.7 };
  const follower = new PathFollowerNode(bus, { pathTopic: 'r/path', poseTopic: 'r/pose', cmdTopic: 'r/cmd', geometry, maxDeviationM: 0.5, onAbort: (a) => aborts.push(a) });
  bus.publish('r/path', { path: [[0, 0], [1, 0], [2, 0]] });
  bus.publish('r/pose', { x: 0.2, y: 0.1, theta: 0 });
  check('on-path pose drives', cmds.length === 1 && (cmds[0].left !== 0 || cmds[0].right !== 0) && aborts.length === 0);
  bus.publish('r/pose', { x: 1, y: 0.9, theta: 0 });
  check('pose 0.9 m off the path -> zero velocity + onAbort(pathDeviation)', cmds.at(-1).left === 0 && cmds.at(-1).right === 0 && aborts.length === 1 && aborts[0].reason === 'pathDeviation' && near(aborts[0].distance, 0.9));
  const n = cmds.length;
  bus.publish('r/pose', { x: 1, y: 0.9, theta: 0 });
  check('after abort the path is dropped (no more commands)', cmds.length === n);
  follower.stop();
  bus.close();
}

// --- 3. Vda5050Node abortOrder + estop ---
{
  const bus = new LocalBus('abort-smoke');
  const published = [];
  const mqtt = { publish: (t, p) => published.push({ t, m: JSON.parse(p) }), subscribe: () => () => {} };
  const node = new Vda5050Node(bus, mqtt, { serialNumber: 'r', poseTopic: 'r/pose', pathTopic: 'r/path', cmdTopic: 'r/cmd', estopTopic: 'r/estop', stateIntervalMs: 0 });
  const paths = [];
  bus.subscribe('r/path', (m) => paths.push(m));
  node._onOrder(JSON.stringify(pathToOrder([[0, 0], [1, 0]], { orderId: 'o1', orderUpdateId: 0 })));
  check('order accepted', node.stateMessage().nodeStates.length === 2);
  node.abortOrder('pathDeviation', 'left the path by 0.9 m');
  const st = published.filter((p) => p.t.endsWith('/state')).at(-1).m;
  check('abortOrder: nodes dropped, FATAL pathDeviation error, path [] sent', st.nodeStates.length === 0 && st.errors.some((e) => e.errorType === 'pathDeviation' && e.errorLevel === 'FATAL') && paths.at(-1).path.length === 0 && st.driving === false);
  bus.publish('r/estop', { active: true });
  check('estop topic -> safetyState.eStop MANUAL + stop', node.stateMessage().safetyState.eStop === 'MANUAL');
  bus.publish('r/estop', { active: false });
  check('estop released -> NONE', node.stateMessage().safetyState.eStop === 'NONE');
  node.close();
  bus.close();
}

console.log(failures === 0 ? '\nall vps-correction / safety smoke checks passed' : `\n${failures} check(s) FAILED`);
process.exit(failures === 0 ? 0 : 1);
