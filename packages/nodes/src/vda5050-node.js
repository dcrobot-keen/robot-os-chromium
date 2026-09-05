// Vda5050Node -- the robot-side VDA5050 adapter. Translates between this
// repo's LocalBus topics (<robotId>/pose, <robotId>/path, <robotId>/drive/
// cmd_vel) and the five VDA5050 MQTT topics, so an RCS (pathfinder's
// server/vda5050.mjs today) can drive and watch the robot without knowing
// anything about the bus. doc/vda5050-rcs.md in the workspace root has the
// design; vda5050.js next door has the pure message maths.
//
// The MQTT client is INJECTED as a two-method object
//   { publish(topic, payloadString, { qos, retain }), subscribe(topic, cb) }
// so this node has no dependency: sim-driver hands it a wrapped `mqtt`
// package client (adaptMqttJsClient below), a browser page can hand it the
// same wrapper around mqtt.min.js, and the smoke test hands it an in-memory
// fake. Nothing in here touches the network directly.
//
//   bus <robotId>/pose            --> visualization (<= 1/visualizationIntervalMs Hz)
//                                 --> state (every stateIntervalMs, plus on every change)
//   MQTT order                    --> bus <robotId>/path { path }
//   MQTT instantActions           --> cancelOrder / stopPause / startPause
//   construct / close / last will --> connection ONLINE / OFFLINE / CONNECTIONBROKEN (retained)
import {
  OrderTracker,
  SUPPORTED_INSTANT_ACTIONS,
  instantActionsOf,
  vda5050Header,
  vda5050Topic,
  velocityBetween,
} from './vda5050.js';

const CONNECTION_QOS = 1;

export class Vda5050Node {
  /**
   * @param {{publish: Function, subscribe: Function}} bus   LocalBus (or compatible)
   * @param {{publish: Function, subscribe: Function}} mqtt  injected MQTT client wrapper
   */
  constructor(bus, mqtt, options = {}) {
    const {
      serialNumber,
      manufacturer = 'dcrobot',
      interfaceName,
      majorVersion,
      mapId = 'default',
      poseTopic,
      pathTopic,
      cmdTopic,
      stateIntervalMs = 1000, // 0 disables the periodic state timer (event-driven only; tests)
      visualizationIntervalMs = 100,
      nodeReachedM = 0.35,
      battery = null, // optional () => { batteryCharge, charging, ... }; omitted from state when null
      now = () => Date.now(),
      log = () => {},
    } = options;
    if (!serialNumber || !poseTopic || !pathTopic || !cmdTopic) {
      throw new Error('Vda5050Node requires serialNumber, poseTopic, pathTopic, cmdTopic');
    }
    this._bus = bus;
    this._mqtt = mqtt;
    this._ids = { interfaceName, majorVersion, manufacturer, serialNumber };
    this._mapId = mapId;
    this._pathTopic = pathTopic;
    this._cmdTopic = cmdTopic;
    this._visIntervalMs = visualizationIntervalMs;
    this._battery = battery;
    this._now = now;
    this._log = log;

    this._tracker = new OrderTracker({ nodeReachedM });
    this._headerIds = new Map(); // topic name -> next headerId
    this._pose = null;
    this._poseAt = 0;
    this._velocity = { vx: 0, vy: 0, omega: 0 };
    this._lastVisAt = -Infinity;
    this._paused = false;
    this._pausedPath = null;
    this._errors = [];
    this._actionStates = [];
    this._closed = false;

    this._topics = Object.fromEntries(
      ['connection', 'state', 'visualization', 'order', 'instantActions'].map((n) => [n, vda5050Topic(this._ids, n)])
    );

    this._unsubPose = bus.subscribe(poseTopic, (pose) => this._onPose(pose));
    this._unsubOrder = mqtt.subscribe(this._topics.order, (_topic, payload) => this._onOrder(payload));
    this._unsubActions = mqtt.subscribe(this._topics.instantActions, (_topic, payload) => this._onInstantActions(payload));

    this._publish('connection', { connectionState: 'ONLINE' }, { qos: CONNECTION_QOS, retain: true });
    this._stateTimer = stateIntervalMs > 0 ? setInterval(() => this.publishState(), stateIntervalMs) : null;
    log(`VDA5050 online as ${this._topics.state.replace(/\/state$/, '')} (mapId ${mapId})`);
  }

  get topics() {
    return { ...this._topics };
  }

  get mapId() {
    return this._mapId;
  }

  /** Change the mapId reported in agvPosition (e.g. once the simulator tells us its world name). */
  setMapId(mapId) {
    if (typeof mapId === 'string' && mapId && mapId !== this._mapId) {
      this._mapId = mapId;
      this._log(`mapId -> ${mapId}`);
      this.publishState();
    }
  }

  /** What to register as the MQTT client's last will: CONNECTIONBROKEN, retained. */
  static lastWill(ids) {
    return {
      topic: vda5050Topic(ids, 'connection'),
      payload: JSON.stringify({
        ...vda5050Header({ headerId: 0, manufacturer: ids.manufacturer, serialNumber: ids.serialNumber }),
        connectionState: 'CONNECTIONBROKEN',
      }),
      qos: CONNECTION_QOS,
      retain: true,
    };
  }

  // --- outgoing ---------------------------------------------------------

  _header(name) {
    const id = this._headerIds.get(name) ?? 0;
    this._headerIds.set(name, id + 1);
    return vda5050Header({
      headerId: id,
      manufacturer: this._ids.manufacturer,
      serialNumber: this._ids.serialNumber,
      timestamp: new Date(this._now()).toISOString(),
    });
  }

  _publish(name, body, opts = { qos: 0, retain: false }) {
    const message = { ...this._header(name), ...body };
    this._mqtt.publish(this._topics[name], JSON.stringify(message), opts);
    return message;
  }

  _agvPosition() {
    if (!this._pose) return { x: 0, y: 0, theta: 0, mapId: this._mapId, positionInitialized: false };
    return { x: this._pose.x, y: this._pose.y, theta: this._pose.theta, mapId: this._mapId, positionInitialized: true };
  }

  /** Build the state message (exported for tests; publishState() sends it). */
  stateMessage() {
    const moving = Math.abs(this._velocity.vx) > 0.01 || Math.abs(this._velocity.omega) > 0.01;
    const state = {
      ...this._tracker.snapshot(),
      driving: !this._paused && (moving || this._tracker.hasActiveOrder),
      paused: this._paused,
      newBaseRequest: false,
      agvPosition: this._agvPosition(),
      velocity: { ...this._velocity },
      actionStates: this._actionStates.map((a) => ({ ...a })),
      operatingMode: 'AUTOMATIC',
      errors: this._errors.map((e) => ({ ...e })),
      safetyState: { eStop: 'NONE', fieldViolation: false },
    };
    if (this._battery) state.batteryState = this._battery();
    return state;
  }

  publishState() {
    if (this._closed) return null;
    return this._publish('state', this.stateMessage());
  }

  publishVisualization() {
    if (this._closed) return null;
    return this._publish('visualization', { agvPosition: this._agvPosition(), velocity: { ...this._velocity } });
  }

  // --- incoming: bus pose -------------------------------------------------

  _onPose(pose) {
    const t = this._now();
    if (this._pose) this._velocity = velocityBetween(this._pose, pose, (t - this._poseAt) / 1000);
    this._pose = { x: pose.x, y: pose.y, theta: pose.theta };
    this._poseAt = t;

    const reached = this._paused ? 0 : this._tracker.advance(pose);
    if (reached > 0) {
      this._log(`reached ${this._tracker.lastNodeId} (${this._tracker.nodeStates.length} nodes left)`);
      this.publishState();
    }
    if (t - this._lastVisAt >= this._visIntervalMs) {
      this._lastVisAt = t;
      this.publishVisualization();
    }
  }

  // --- incoming: MQTT order / instantActions ------------------------------

  _pushError(errorType, errorDescription, errorLevel = 'WARNING', references = []) {
    this._errors.push({ errorType, errorLevel, errorDescription, errorReferences: references });
    if (this._errors.length > 10) this._errors.shift();
  }

  _onOrder(payload) {
    let order;
    try {
      order = JSON.parse(payload);
    } catch (err) {
      this._pushError('validationError', `order is not JSON: ${err.message}`);
      this.publishState();
      return;
    }
    const result = this._tracker.accept(order);
    if (!result.ok) {
      this._pushError(result.error.errorType, result.error.errorDescription, 'WARNING', [
        { referenceKey: 'orderId', referenceValue: String(order?.orderId ?? '') },
      ]);
      this._log(`order rejected: ${result.error.errorDescription}`);
      this.publishState();
      return;
    }
    // A new/updated order clears order-related errors and any pause.
    this._errors = this._errors.filter((e) => !['orderUpdateError', 'validationError'].includes(e.errorType));
    this._paused = false;
    this._pausedPath = null;
    this._log(`order ${order.orderId}/${order.orderUpdateId}: ${result.path.length} waypoints`);
    this._bus.publish(this._pathTopic, { path: result.path });
    this.publishState();
  }

  _stopDriving() {
    this._bus.publish(this._pathTopic, { path: [] });
    this._bus.publish(this._cmdTopic, { left: 0, right: 0 });
  }

  _onInstantActions(payload) {
    let message;
    try {
      message = JSON.parse(payload);
    } catch (err) {
      this._pushError('validationError', `instantActions is not JSON: ${err.message}`);
      this.publishState();
      return;
    }
    for (const action of instantActionsOf(message)) {
      const actionId = String(action.actionId ?? `${action.actionType}-${this._now()}`);
      const done = (status, resultDescription) => {
        this._actionStates = this._actionStates.filter((a) => a.actionId !== actionId).slice(-9);
        this._actionStates.push({ actionId, actionType: action.actionType, actionStatus: status, ...(resultDescription ? { resultDescription } : {}) });
      };
      switch (action.actionType) {
        case 'cancelOrder':
          this._tracker.cancel();
          this._paused = false;
          this._pausedPath = null;
          this._stopDriving();
          done('FINISHED');
          break;
        case 'stopPause':
          if (!this._paused) {
            this._pausedPath = this._tracker.remainingPath();
            this._paused = true;
            this._stopDriving();
          }
          done('FINISHED');
          break;
        case 'startPause':
          if (this._paused) {
            this._paused = false;
            if (this._pausedPath?.length) this._bus.publish(this._pathTopic, { path: this._pausedPath });
            this._pausedPath = null;
          }
          done('FINISHED');
          break;
        default:
          this._pushError('unsupportedAction', `actionType "${action.actionType}" is not supported (supported: ${SUPPORTED_INSTANT_ACTIONS.join(', ')})`, 'WARNING', [
            { referenceKey: 'actionId', referenceValue: actionId },
          ]);
          done('FAILED', 'unsupported actionType');
      }
      this._log(`instantAction ${action.actionType} -> ${this._actionStates.at(-1).actionStatus}`);
    }
    this.publishState();
  }

  close() {
    if (this._closed) return;
    if (this._stateTimer) clearInterval(this._stateTimer);
    this._publish('connection', { connectionState: 'OFFLINE' }, { qos: CONNECTION_QOS, retain: true });
    this._closed = true;
    this._unsubPose?.();
    this._unsubOrder?.();
    this._unsubActions?.();
  }
}

/**
 * Wrap an mqtt.js-style client (npm `mqtt` in Node, mqtt.min.js in the
 * browser: publish(topic, payload, opts), subscribe(topic), on('message'))
 * into the { publish, subscribe } shape Vda5050Node takes. Exact-topic
 * dispatch only -- we never subscribe with wildcards on the robot side.
 */
export function adaptMqttJsClient(client) {
  const handlers = new Map(); // topic -> Set<cb>
  client.on('message', (topic, payload) => {
    const cbs = handlers.get(topic);
    if (!cbs) return;
    const text = typeof payload === 'string' ? payload : new TextDecoder().decode(payload);
    for (const cb of cbs) cb(topic, text);
  });
  return {
    publish(topic, payload, opts = {}) {
      client.publish(topic, payload, { qos: opts.qos ?? 0, retain: opts.retain ?? false });
    },
    subscribe(topic, cb) {
      let cbs = handlers.get(topic);
      if (!cbs) {
        cbs = new Set();
        handlers.set(topic, cbs);
        client.subscribe(topic);
      }
      cbs.add(cb);
      return () => {
        cbs.delete(cb);
        if (cbs.size === 0) {
          handlers.delete(topic);
          client.unsubscribe?.(topic);
        }
      };
    },
  };
}
