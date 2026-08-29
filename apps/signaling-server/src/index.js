// Signaling server — the only piece of the stack that sits in the middle of
// a connection instead of at one end of it, and it deliberately does as
// little as possible. It relays WebRTC handshake messages (SDP offers /
// answers, ICE candidates) between a robot's "host" peer and one or more
// "operator" peers so they can open a direct data channel, and then it is
// out of the loop entirely — robot commands and telemetry never pass
// through here (see research.md, Layer 07; plan.md Phase 5).
//
// Model: one "room" per robot id. A host registers itself for a robot id;
// operators register against the same id. Any `signal` message from a peer
// is forwarded verbatim to the other peer(s) in that room, tagged with the
// sender's id. That's the whole data path. `list` returns the known robots
// and is the seed of the Phase 6 fleet registry — nothing downstream uses
// it yet.
//
// Run: node src/index.js   (or `npm start`).  Port: SIGNALING_PORT (9770).

import { WebSocketServer } from 'ws';

const PORT = Number(process.env.SIGNALING_PORT || 9770);

// robot id -> { host: peer|null, operators: Set<peer>, manifest: object|null }
const rooms = new Map();
let nextPeerId = 1;

function log(msg) {
  console.log(`[signaling ${new Date().toISOString()}] ${msg}`);
}

function roomFor(robot) {
  let room = rooms.get(robot);
  if (!room) {
    room = { host: null, operators: new Set(), manifest: null };
    rooms.set(robot, room);
  }
  return room;
}

function peersInRoom(room) {
  return [room.host, ...room.operators].filter(Boolean);
}

function send(peer, msg) {
  if (peer.ws.readyState === peer.ws.OPEN) peer.ws.send(JSON.stringify(msg));
}

function robotList() {
  return [...rooms.entries()].map(([robot, room]) => ({
    robot,
    online: !!room.host,
    operators: room.operators.size,
    manifest: room.manifest,
  }));
}

function handleHello(peer, msg) {
  if (peer.robot) return send(peer, { type: 'error', message: 'already registered' });
  if (msg.role !== 'host' && msg.role !== 'operator') {
    return send(peer, { type: 'error', message: `unknown role: ${msg.role}` });
  }
  if (typeof msg.robot !== 'string' || !msg.robot) {
    return send(peer, { type: 'error', message: 'hello needs a robot id' });
  }

  const room = roomFor(msg.robot);
  if (msg.role === 'host' && room.host) {
    return send(peer, { type: 'error', message: `robot ${msg.robot} already has a host` });
  }

  peer.role = msg.role;
  peer.robot = msg.robot;
  if (msg.role === 'host') {
    room.host = peer;
    if (msg.manifest && typeof msg.manifest === 'object') room.manifest = msg.manifest;
  } else {
    room.operators.add(peer);
  }
  log(`peer ${peer.id} registered as ${peer.role} for ${peer.robot} (room now: host=${!!room.host}, operators=${room.operators.size})`);

  // Tell the newcomer who's already here, so it knows whether to start the
  // WebRTC offer now or wait for the other side to show up.
  const others = peersInRoom(room).filter((p) => p !== peer);
  send(peer, {
    type: 'ready',
    peerId: peer.id,
    robot: peer.robot,
    role: peer.role,
    peers: others.map((p) => ({ peerId: p.id, role: p.role })),
  });
  for (const other of others) send(other, { type: 'peer-joined', peerId: peer.id, role: peer.role });
}

function handleSignal(peer, msg) {
  if (!peer.robot) return send(peer, { type: 'error', message: 'signal before hello' });
  const room = rooms.get(peer.robot);
  if (!room) return;

  // With `to`, unicast to that peer; without, forward to everyone else in
  // the room (fine for the one-host/one-operator case).
  const targets = msg.to
    ? peersInRoom(room).filter((p) => p.id === msg.to)
    : peersInRoom(room).filter((p) => p !== peer);
  for (const target of targets) send(target, { type: 'signal', from: peer.id, data: msg.data });
}

function handleClose(peer) {
  if (!peer.robot) return;
  const room = rooms.get(peer.robot);
  if (!room) return;

  if (room.host === peer) room.host = null;
  room.operators.delete(peer);
  log(`peer ${peer.id} (${peer.role}) left ${peer.robot} (room now: host=${!!room.host}, operators=${room.operators.size})`);

  for (const other of peersInRoom(room)) send(other, { type: 'peer-left', peerId: peer.id, role: peer.role });
  if (!room.host && room.operators.size === 0) rooms.delete(peer.robot);
}

const wss = new WebSocketServer({ port: PORT });

wss.on('connection', (ws) => {
  const peer = { id: nextPeerId++, ws, role: null, robot: null };
  log(`peer ${peer.id} connected`);

  ws.on('message', (raw) => {
    let msg;
    try {
      msg = JSON.parse(raw.toString());
    } catch {
      return send(peer, { type: 'error', message: 'invalid JSON' });
    }
    switch (msg.type) {
      case 'hello': return handleHello(peer, msg);
      case 'signal': return handleSignal(peer, msg);
      case 'list': return send(peer, { type: 'robots', robots: robotList() });
      default: return send(peer, { type: 'error', message: `unknown message type: ${msg.type}` });
    }
  });

  ws.on('close', () => handleClose(peer));
  ws.on('error', () => {}); // an operator/host that crashes just drops the socket
});

log(`listening on ws://127.0.0.1:${PORT}`);
