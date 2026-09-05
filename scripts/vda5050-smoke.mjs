// Smoke test for Vda5050Node + vda5050.js -- no broker, no simulator. An
// in-memory fake stands in for the MQTT client (the node only ever sees
// { publish, subscribe }), LocalBus is the real one, time is injected so
// velocity/throttling are deterministic.
//
//   node scripts/vda5050-smoke.mjs
import { LocalBus } from '@ros-chromium/bus';
import {
  Vda5050Node,
  adaptMqttJsClient,
  OrderTracker,
  orderToPath,
  pathToOrder,
  parseVda5050Topic,
  vda5050Topic,
  velocityBetween,
  instantActionsOf,
} from '@ros-chromium/nodes';

let failures = 0;
function check(name, cond, extra = '') {
  console.log(`${cond ? 'PASS' : 'FAIL'}  ${name}${extra ? '  (' + extra + ')' : ''}`);
  if (!cond) failures++;
}

// --- fake MQTT: records publishes, delivers injected messages to subscribers ---
function fakeMqtt() {
  const published = [];
  const subs = new Map();
  return {
    published,
    publish(topic, payload, opts) {
      published.push({ topic, message: JSON.parse(payload), opts });
    },
    subscribe(topic, cb) {
      if (!subs.has(topic)) subs.set(topic, new Set());
      subs.get(topic).add(cb);
      return () => subs.get(topic).delete(cb);
    },
    inject(topic, message) {
      for (const cb of subs.get(topic) ?? []) cb(topic, typeof message === 'string' ? message : JSON.stringify(message));
    },
    last(name) {
      return published.filter((p) => p.topic.endsWith('/' + name)).at(-1)?.message;
    },
    count(name) {
      return published.filter((p) => p.topic.endsWith('/' + name)).length;
    },
  };
}

const ids = { manufacturer: 'dcrobot', serialNumber: 'tb3-sim-01' };

// --- 1. topics + pure helpers ---
{
  const t = vda5050Topic(ids, 'state');
  check('topic uses uagv/v2 defaults', t === 'uagv/v2/dcrobot/tb3-sim-01/state', t);
  const parsed = parseVda5050Topic(t);
  check('topic parses back', parsed?.serialNumber === 'tb3-sim-01' && parsed.name === 'state' && parsed.majorVersion === 'v2');
  check('foreign topic rejected', parseVda5050Topic('uagv/v2/x/y/nope') === null && parseVda5050Topic('a/b') === null);

  const path = [[0, 0], [1, 0], [1, 1]];
  const order = pathToOrder(path, { orderId: 'o1', orderUpdateId: 0, mapId: 'default' });
  check('pathToOrder: 3 nodes, 2 edges', order.nodes.length === 3 && order.edges.length === 2);
  check('sequenceIds alternate node/edge', order.nodes.map((n) => n.sequenceId).join() === '0,2,4' && order.edges.map((e) => e.sequenceId).join() === '1,3');
  check('edges connect consecutive nodes', order.edges[1].startNodeId === 'n1' && order.edges[1].endNodeId === 'n2');
  const back = orderToPath(order);
  check('orderToPath round-trips the coordinates', JSON.stringify(back.path) === JSON.stringify(path));
  const shuffled = { ...order, nodes: [order.nodes[2], order.nodes[0], order.nodes[1]] };
  check('orderToPath sorts by sequenceId', JSON.stringify(orderToPath(shuffled).path) === JSON.stringify(path));
  const horizon = { ...order, nodes: order.nodes.map((n, i) => (i === 2 ? { ...n, released: false } : n)) };
  check('unreleased horizon nodes are skipped', orderToPath(horizon).path.length === 2);
  let threw = null;
  try { orderToPath({ orderId: 'x', orderUpdateId: 0, nodes: [{ nodeId: 'a', sequenceId: 0 }] }); } catch (e) { threw = e; }
  check('node without nodePosition -> validationError', threw?.errorType === 'validationError');
  threw = null;
  try { orderToPath({ orderId: 'x', orderUpdateId: -1, nodes: [] }); } catch (e) { threw = e; }
  check('bad orderUpdateId -> validationError', threw?.errorType === 'validationError');

  const v = velocityBetween({ x: 0, y: 0, theta: 0 }, { x: 0.5, y: 0, theta: 0 }, 0.5);
  check('velocityBetween: vx 1 m/s straight ahead', Math.abs(v.vx - 1) < 1e-9 && Math.abs(v.vy) < 1e-9 && v.omega === 0);
  const turn = velocityBetween({ x: 0, y: 0, theta: 0 }, { x: 0, y: 0.5, theta: Math.PI / 2 }, 0.5);
  check('velocityBetween: displacement projected on the new heading, omega pi rad/s', Math.abs(turn.vx - 1) < 1e-9 && Math.abs(turn.omega - Math.PI) < 1e-9);
  const wrap = velocityBetween({ x: 0, y: 0, theta: 3.1 }, { x: 0, y: 0, theta: -3.1 }, 1);
  check('velocityBetween wraps theta', Math.abs(wrap.omega - (2 * Math.PI - 6.2)) < 1e-9, wrap.omega.toFixed(3));
  check('instantActionsOf accepts 2.0 `actions` and 2.1 `instantActions`',
    instantActionsOf({ actions: [{ actionType: 'a' }] }).length === 1 && instantActionsOf({ instantActions: [{ actionType: 'b' }, { nope: 1 }] }).length === 1);
}

// --- 2. OrderTracker rules ---
{
  const tr = new OrderTracker({ nodeReachedM: 0.3 });
  const o1 = pathToOrder([[0, 0], [2, 0], [2, 2]], { orderId: 'A', orderUpdateId: 0 });
  check('tracker accepts first order', tr.accept(o1).ok && tr.nodeStates.length === 3 && tr.edgeStates.length === 2);
  check('tracker rejects same orderUpdateId', tr.accept(o1).ok === false && tr.accept(o1).error.errorType === 'orderUpdateError');
  check('tracker rejects older orderUpdateId', tr.accept({ ...o1, orderUpdateId: -0 }).ok === false);
  check('advance: far away reaches nothing', tr.advance({ x: 5, y: 5, theta: 0 }) === 0 && tr.lastNodeId === '');
  check('advance: at n0 reaches n0', tr.advance({ x: 0.1, y: 0.1, theta: 0 }) === 1 && tr.lastNodeId === 'n0' && tr.nodeStates.length === 2);
  check('edges before lastNode dropped only after their end node', tr.edgeStates.length === 2);
  check('advance: at n1 reaches n1 and drops e0', tr.advance({ x: 2, y: 0.05, theta: 0 }) === 1 && tr.lastNodeId === 'n1' && tr.edgeStates.map((e) => e.edgeId).join() === 'e1');
  check('remainingPath = unreached nodes', JSON.stringify(tr.remainingPath()) === '[[2,2]]');
  const upd = tr.accept({ ...pathToOrder([[2, 0], [2, 2], [0, 2]], { orderId: 'A', orderUpdateId: 1 }) });
  check('newer orderUpdateId replaces nodes but keeps lastNodeId', upd.ok && tr.nodeStates.length === 3 && tr.lastNodeId === 'n1');
  check('new orderId resets lastNodeId', tr.accept(pathToOrder([[0, 0]], { orderId: 'B', orderUpdateId: 0 })).ok && tr.lastNodeId === '' && tr.orderId === 'B');
  tr.cancel();
  check('cancel empties nodes but keeps orderId', tr.nodeStates.length === 0 && tr.orderId === 'B' && !tr.hasActiveOrder);
}

// --- 3. Vda5050Node end to end over LocalBus + fake MQTT ---
{
  let clock = 1_000_000;
  const now = () => clock;
  const bus = new LocalBus('vda5050-smoke');
  const mqtt = fakeMqtt();
  const pathMsgs = [];
  const cmdMsgs = [];
  bus.subscribe('tb3-sim-01/path', (m) => pathMsgs.push(m));
  bus.subscribe('tb3-sim-01/drive/cmd_vel', (m) => cmdMsgs.push(m));

  const node = new Vda5050Node(bus, mqtt, {
    ...ids,
    mapId: 'default',
    poseTopic: 'tb3-sim-01/pose',
    pathTopic: 'tb3-sim-01/path',
    cmdTopic: 'tb3-sim-01/drive/cmd_vel',
    stateIntervalMs: 0,
    visualizationIntervalMs: 100,
    nodeReachedM: 0.3,
    now,
  });

  const conn = mqtt.published[0];
  check('connection ONLINE retained qos1 on construct', conn.topic === 'uagv/v2/dcrobot/tb3-sim-01/connection' && conn.message.connectionState === 'ONLINE' && conn.opts.retain === true && conn.opts.qos === 1);
  check('header fields present', conn.message.headerId === 0 && conn.message.version === '2.0.0' && conn.message.manufacturer === 'dcrobot' && typeof conn.message.timestamp === 'string');

  const s0 = node.stateMessage();
  check('state before any pose: positionInitialized false, not driving', s0.agvPosition.positionInitialized === false && s0.driving === false && s0.orderId === '' && Array.isArray(s0.errors));
  check('batteryState omitted when no provider', !('batteryState' in s0));

  // pose -> visualization (throttled)
  bus.publish('tb3-sim-01/pose', { x: 0, y: 0, theta: 0 });
  clock += 50;
  bus.publish('tb3-sim-01/pose', { x: 0.05, y: 0, theta: 0 });
  clock += 50;
  bus.publish('tb3-sim-01/pose', { x: 0.1, y: 0, theta: 0 });
  check('visualization throttled to the interval', mqtt.count('visualization') === 2, `${mqtt.count('visualization')} published for 3 poses in 100ms`);
  const vis = mqtt.last('visualization');
  check('visualization carries agvPosition+mapId+velocity', vis.agvPosition.x === 0.1 && vis.agvPosition.mapId === 'default' && Math.abs(vis.velocity.vx - 1) < 1e-9);

  // order -> bus path + state
  const order = pathToOrder([[0.1, 0], [1, 0], [1, 1]], { orderId: 'o1', orderUpdateId: 0, mapId: 'default' });
  mqtt.inject(node.topics.order, order);
  check('order -> bus path', pathMsgs.length === 1 && pathMsgs[0].path.length === 3 && pathMsgs[0].path[2][1] === 1);
  let st = mqtt.last('state');
  check('state after order: orderId, 3 nodeStates, driving', st.orderId === 'o1' && st.nodeStates.length === 3 && st.driving === true && st.lastNodeId === '');

  // progress
  bus.publish('tb3-sim-01/pose', { x: 0.15, y: 0.02, theta: 0 }); // within 0.3 of n0
  st = mqtt.last('state');
  check('reaching n0 publishes state with lastNodeId n0', st.lastNodeId === 'n0' && st.lastNodeSequenceId === 0 && st.nodeStates.length === 2);
  clock += 1000;
  bus.publish('tb3-sim-01/pose', { x: 1.02, y: 0.01, theta: 0 });
  st = mqtt.last('state');
  check('reaching n1 drops e0', st.lastNodeId === 'n1' && st.edgeStates.map((e) => e.edgeId).join() === 'e1');

  // duplicate update rejected
  const before = pathMsgs.length;
  mqtt.inject(node.topics.order, order);
  st = mqtt.last('state');
  check('stale orderUpdateId: no path, orderUpdateError reported', pathMsgs.length === before && st.errors.some((e) => e.errorType === 'orderUpdateError' && e.errorReferences[0].referenceValue === 'o1'));

  // pause / resume
  mqtt.inject(node.topics.instantActions, { actions: [{ actionId: 'p1', actionType: 'stopPause' }] });
  check('stopPause: empty path + zero cmd_vel', pathMsgs.at(-1).path.length === 0 && cmdMsgs.at(-1).left === 0 && cmdMsgs.at(-1).right === 0);
  st = mqtt.last('state');
  check('stopPause: paused true, not driving, action FINISHED', st.paused === true && st.driving === false && st.actionStates.some((a) => a.actionId === 'p1' && a.actionStatus === 'FINISHED'));
  bus.publish('tb3-sim-01/pose', { x: 1.0, y: 1.0, theta: 0 }); // at n2 but paused -> must not count
  check('paused: no node progress', node.stateMessage().lastNodeId === 'n1');
  mqtt.inject(node.topics.instantActions, { instantActions: [{ actionId: 'p2', actionType: 'startPause' }] });
  check('startPause: remaining path re-sent (n2 only)', pathMsgs.at(-1).path.length === 1 && pathMsgs.at(-1).path[0][0] === 1 && pathMsgs.at(-1).path[0][1] === 1);
  check('startPause: unpaused', mqtt.last('state').paused === false);

  // newer update clears errors
  mqtt.inject(node.topics.order, { ...pathToOrder([[1, 1], [0, 1]], { orderId: 'o1', orderUpdateId: 1 }) });
  st = mqtt.last('state');
  check('newer orderUpdateId accepted and clears orderUpdateError', st.orderUpdateId === 1 && !st.errors.some((e) => e.errorType === 'orderUpdateError') && pathMsgs.at(-1).path.length === 2);

  // cancel + unsupported
  mqtt.inject(node.topics.instantActions, { actions: [{ actionId: 'c1', actionType: 'cancelOrder' }, { actionId: 'x1', actionType: 'initPosition' }] });
  st = mqtt.last('state');
  check('cancelOrder: nodes empty, orderId kept, path [] sent', st.nodeStates.length === 0 && st.orderId === 'o1' && pathMsgs.at(-1).path.length === 0);
  check('unsupported action: FAILED + unsupportedAction warning', st.actionStates.some((a) => a.actionId === 'x1' && a.actionStatus === 'FAILED') && st.errors.some((e) => e.errorType === 'unsupportedAction'));

  // garbage
  const errsBefore = st.errors.length;
  mqtt.inject(node.topics.order, 'not json {');
  check('non-JSON order -> validationError, no path', mqtt.last('state').errors.length === errsBefore + 1 && pathMsgs.at(-1).path.length === 0);

  // battery provider
  const withBattery = new Vda5050Node(new LocalBus('vda5050-smoke-b'), fakeMqtt(), {
    ...ids, serialNumber: 'tb3-sim-02', poseTopic: 'p', pathTopic: 'q', cmdTopic: 'c', stateIntervalMs: 0, battery: () => ({ batteryCharge: 42, charging: false }),
  });
  check('battery provider lands in state', withBattery.stateMessage().batteryState.batteryCharge === 42);
  withBattery.close();

  // close + last will
  node.close();
  const off = mqtt.last('connection');
  check('close publishes OFFLINE retained', off.connectionState === 'OFFLINE' && mqtt.published.at(-1).opts.retain === true);
  const will = Vda5050Node.lastWill(ids);
  check('lastWill is CONNECTIONBROKEN on the connection topic', will.topic.endsWith('/connection') && JSON.parse(will.payload).connectionState === 'CONNECTIONBROKEN' && will.retain === true);
  const stateCountAtClose = mqtt.count('state');
  mqtt.inject(node.topics.order, order);
  check('closed node ignores further orders', mqtt.count('state') === stateCountAtClose);
  bus.close();
}

// --- 4. adaptMqttJsClient dispatches exact topics from an mqtt.js-like client ---
{
  const handlers = {};
  const calls = { subscribe: [], publish: [], unsubscribe: [] };
  const fakeClient = {
    on(evt, cb) { handlers[evt] = cb; },
    subscribe(t) { calls.subscribe.push(t); },
    unsubscribe(t) { calls.unsubscribe.push(t); },
    publish(t, p, o) { calls.publish.push({ t, p, o }); },
  };
  const wrapped = adaptMqttJsClient(fakeClient);
  const got = [];
  const unsub = wrapped.subscribe('a/b', (t, p) => got.push([t, p]));
  wrapped.subscribe('a/b', () => got.push('second'));
  check('wrapper subscribes the client once per topic', calls.subscribe.length === 1);
  handlers.message('a/b', new TextEncoder().encode('{"x":1}'));
  handlers.message('a/c', 'ignored');
  check('wrapper delivers Buffer payloads as text to every handler', got.length === 2 && got[0][1] === '{"x":1}');
  wrapped.publish('a/b', 'hi', { qos: 1, retain: true });
  check('wrapper forwards qos/retain', calls.publish[0].o.qos === 1 && calls.publish[0].o.retain === true);
  unsub();
  check('unsubscribe keeps the client subscription while another handler remains', calls.unsubscribe.length === 0);
}

console.log(failures === 0 ? '\nall vda5050 smoke checks passed' : `\n${failures} check(s) FAILED`);
process.exit(failures === 0 ? 0 : 1);
