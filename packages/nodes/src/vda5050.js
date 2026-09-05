// VDA5050 2.x message helpers -- the pure, bus-free half of Vda5050Node
// (vda5050-node.js). Topic naming, header building, order <-> waypoint path
// conversion and the order-update bookkeeping the spec asks an AGV to do.
// Design + the values we fixed (interfaceName, manufacturer, mapId, QoS):
// doc/vda5050-rcs.md in the workspace root.
//
// Only the subset our loop needs is implemented: released nodes with
// nodePosition, no horizon, no edge trajectories, no actions on nodes.
// Anything the RCS sends beyond that is either ignored (unknown fields) or
// reported back on `state.errors` (unsupported instant actions).

export const VDA5050_VERSION = '2.0.0';
export const DEFAULT_INTERFACE_NAME = 'uagv';
export const DEFAULT_MAJOR_VERSION = 'v2';

export const TOPIC_NAMES = ['connection', 'state', 'visualization', 'order', 'instantActions', 'factsheet'];

/** `uagv/v2/<manufacturer>/<serialNumber>/<name>` */
export function vda5050Topic({ interfaceName = DEFAULT_INTERFACE_NAME, majorVersion = DEFAULT_MAJOR_VERSION, manufacturer, serialNumber }, name) {
  if (!manufacturer || !serialNumber) throw new Error('vda5050Topic needs manufacturer and serialNumber');
  if (!TOPIC_NAMES.includes(name)) throw new Error(`unknown VDA5050 topic name "${name}"`);
  return `${interfaceName}/${majorVersion}/${manufacturer}/${serialNumber}/${name}`;
}

/** Inverse of vda5050Topic; null when the topic isn't five segments ending in a known name. */
export function parseVda5050Topic(topic) {
  const parts = String(topic).split('/');
  if (parts.length !== 5 || !TOPIC_NAMES.includes(parts[4])) return null;
  const [interfaceName, majorVersion, manufacturer, serialNumber, name] = parts;
  return { interfaceName, majorVersion, manufacturer, serialNumber, name };
}

/** The five header fields every VDA5050 message starts with. */
export function vda5050Header({ headerId, manufacturer, serialNumber, version = VDA5050_VERSION, timestamp = new Date().toISOString() }) {
  return { headerId, timestamp, version, manufacturer, serialNumber };
}

function isFiniteNumber(v) {
  return typeof v === 'number' && Number.isFinite(v);
}

class OrderValidationError extends Error {
  constructor(message) {
    super(message);
    this.errorType = 'validationError';
  }
}

/**
 * Validate an order and reduce it to what PathFollowerNode consumes:
 * released nodes in sequenceId order -> [[x, y], ...].
 * Throws (errorType 'validationError') on anything malformed.
 */
export function orderToPath(order) {
  if (!order || typeof order !== 'object') throw new OrderValidationError('order is not an object');
  if (typeof order.orderId !== 'string' || !order.orderId) throw new OrderValidationError('orderId missing');
  if (!Number.isInteger(order.orderUpdateId) || order.orderUpdateId < 0) throw new OrderValidationError('orderUpdateId must be a non-negative integer');
  if (!Array.isArray(order.nodes) || order.nodes.length === 0) throw new OrderValidationError('nodes must be a non-empty array');
  const nodes = order.nodes.slice().sort((a, b) => a.sequenceId - b.sequenceId);
  const released = [];
  for (const node of nodes) {
    if (typeof node.nodeId !== 'string' || !node.nodeId) throw new OrderValidationError('node without nodeId');
    if (!Number.isInteger(node.sequenceId)) throw new OrderValidationError(`node ${node.nodeId}: sequenceId must be an integer`);
    if (node.released === false) continue; // horizon -- we don't drive it
    const p = node.nodePosition;
    if (!p || !isFiniteNumber(p.x) || !isFiniteNumber(p.y)) throw new OrderValidationError(`node ${node.nodeId}: nodePosition.x/y required`);
    released.push(node);
  }
  if (released.length === 0) throw new OrderValidationError('order has no released nodes');
  const edges = Array.isArray(order.edges) ? order.edges.slice().sort((a, b) => a.sequenceId - b.sequenceId) : [];
  return { path: released.map((n) => [n.nodePosition.x, n.nodePosition.y]), nodes: released, edges };
}

/**
 * RCS side: waypoint path -> order. Nodes get even sequenceIds, edges odd,
 * everything released (no horizon). `header` is spread in first so the
 * caller controls headerId/timestamp/manufacturer/serialNumber.
 */
export function pathToOrder(path, { orderId, orderUpdateId = 0, mapId, header = {} }) {
  if (!Array.isArray(path) || path.length === 0) throw new Error('path must be a non-empty [[x, y], ...]');
  if (typeof orderId !== 'string' || !orderId) throw new Error('orderId required');
  const nodes = path.map(([x, y], i) => ({
    nodeId: `n${i}`,
    sequenceId: i * 2,
    released: true,
    nodePosition: { x, y, ...(mapId ? { mapId } : {}) },
    actions: [],
  }));
  const edges = path.slice(1).map((_, i) => ({
    edgeId: `e${i}`,
    sequenceId: i * 2 + 1,
    released: true,
    startNodeId: `n${i}`,
    endNodeId: `n${i + 1}`,
    actions: [],
  }));
  return { ...header, orderId, orderUpdateId, nodes, edges };
}

/**
 * Order bookkeeping per VDA5050 §6.6: which order is active, which nodes
 * remain, what the last reached node was. Position-only progress: a node
 * counts as reached when the pose comes within `nodeReachedM` of it (pure
 * pursuit never lands exactly on a waypoint).
 */
export class OrderTracker {
  constructor({ nodeReachedM = 0.35 } = {}) {
    this.nodeReachedM = nodeReachedM;
    this.orderId = '';
    this.orderUpdateId = 0;
    this.lastNodeId = '';
    this.lastNodeSequenceId = 0;
    this.nodeStates = []; // remaining nodes, in order
    this.edgeStates = []; // remaining edges, in order
    this._path = [];
  }

  /** Returns { ok: true, path } or { ok: false, error: { errorType, errorDescription } }. */
  accept(order) {
    let reduced;
    try {
      reduced = orderToPath(order);
    } catch (err) {
      return { ok: false, error: { errorType: err.errorType ?? 'validationError', errorDescription: err.message } };
    }
    if (order.orderId === this.orderId && order.orderUpdateId <= this.orderUpdateId) {
      return {
        ok: false,
        error: {
          errorType: 'orderUpdateError',
          errorDescription: `orderUpdateId ${order.orderUpdateId} is not newer than ${this.orderUpdateId} for order ${order.orderId}`,
        },
      };
    }
    const newOrder = order.orderId !== this.orderId;
    this.orderId = order.orderId;
    this.orderUpdateId = order.orderUpdateId;
    if (newOrder) {
      this.lastNodeId = '';
      this.lastNodeSequenceId = 0;
    }
    this.nodeStates = reduced.nodes.map((n) => ({
      nodeId: n.nodeId,
      sequenceId: n.sequenceId,
      released: true,
      nodePosition: { x: n.nodePosition.x, y: n.nodePosition.y, ...(n.nodePosition.mapId ? { mapId: n.nodePosition.mapId } : {}) },
    }));
    this.edgeStates = reduced.edges.map((e) => ({ edgeId: e.edgeId, sequenceId: e.sequenceId, released: e.released !== false }));
    this._path = reduced.path;
    return { ok: true, path: reduced.path };
  }

  /**
   * Mark nodes reached by the pose; returns how many were newly reached.
   * Not just the head: pure pursuit cuts corners and a noisy pose estimate can
   * jump, so the robot may pass a node without ever coming within nodeReachedM
   * of it while a LATER node is within reach. Every node up to the farthest
   * one currently within the radius counts as traversed (VDA5050 only needs
   * lastNodeId to move forward, never back).
   */
  advance(pose) {
    let farthest = -1;
    for (let i = 0; i < this.nodeStates.length; i++) {
      const n = this.nodeStates[i];
      if (Math.hypot(pose.x - n.nodePosition.x, pose.y - n.nodePosition.y) <= this.nodeReachedM) farthest = i;
    }
    if (farthest < 0) return 0;
    return this._popThrough(farthest);
  }

  _popThrough(index) {
    const done = this.nodeStates.splice(0, index + 1);
    const last = done[done.length - 1];
    this.lastNodeId = last.nodeId;
    this.lastNodeSequenceId = last.sequenceId;
    this.edgeStates = this.edgeStates.filter((e) => e.sequenceId > last.sequenceId);
    return done.length;
  }

  /** The follower says it is at the end of the path: every remaining node counts as reached. */
  complete() {
    if (this.nodeStates.length === 0) return 0;
    return this._popThrough(this.nodeStates.length - 1);
  }

  /** Waypoints not yet reached (what to re-send after a pause). */
  remainingPath() {
    return this.nodeStates.map((n) => [n.nodePosition.x, n.nodePosition.y]);
  }

  get hasActiveOrder() {
    return this.nodeStates.length > 0;
  }

  /** cancelOrder: keep orderId (the spec says it stays until the next order) but drop all remaining nodes/edges. */
  cancel() {
    this.nodeStates = [];
    this.edgeStates = [];
    this._path = [];
  }

  snapshot() {
    return {
      orderId: this.orderId,
      orderUpdateId: this.orderUpdateId,
      lastNodeId: this.lastNodeId,
      lastNodeSequenceId: this.lastNodeSequenceId,
      nodeStates: this.nodeStates.map((n) => ({ ...n, nodePosition: { ...n.nodePosition } })),
      edgeStates: this.edgeStates.map((e) => ({ ...e })),
    };
  }
}

/** Instant actions we execute; everything else is reported FAILED with an `unsupportedAction` warning. */
export const SUPPORTED_INSTANT_ACTIONS = ['cancelOrder', 'stopPause', 'startPause'];

/** The 2.0 message carries `actions`, 2.1 renamed it to `instantActions`; accept both. */
export function instantActionsOf(message) {
  if (!message || typeof message !== 'object') return [];
  const list = Array.isArray(message.instantActions) ? message.instantActions : Array.isArray(message.actions) ? message.actions : [];
  return list.filter((a) => a && typeof a.actionType === 'string');
}

/** Planar velocity from two timestamped poses (vx along heading, omega), or zeros. */
export function velocityBetween(prev, next, dtSeconds) {
  if (!prev || !next || !(dtSeconds > 0)) return { vx: 0, vy: 0, omega: 0 };
  const dx = next.x - prev.x;
  const dy = next.y - prev.y;
  const cos = Math.cos(next.theta);
  const sin = Math.sin(next.theta);
  let dTheta = next.theta - prev.theta;
  while (dTheta > Math.PI) dTheta -= 2 * Math.PI;
  while (dTheta < -Math.PI) dTheta += 2 * Math.PI;
  return {
    vx: (cos * dx + sin * dy) / dtSeconds,
    vy: (-sin * dx + cos * dy) / dtSeconds,
    omega: dTheta / dtSeconds,
  };
}
