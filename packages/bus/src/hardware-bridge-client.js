// HardwareBridgeClient — the tab-side proxy for hardware-bridge-worker.js.
// Implements the same connect()/send()/encode()/onMessage()/onDisconnect()
// shape as HardwareTransport (see packages/transport/src/index.js), so
// createDriveDevice() and anything else built against that interface works
// unmodified whether it's handed a real WebSocketTransport or one of
// these — swapping the transport implementation without touching the
// layers above it is exactly what that interface is for.
//
// Every tab that constructs one of these shares the same underlying
// hardware connection (there's exactly one real WebSocketTransport, living
// in the worker) instead of each tab opening its own. Encoding happens
// here on the tab side (the worker just forwards the bytes); decoding
// happens in the worker. Both default to the Roboteq codec. A factory
// codec with request/response state (TB3's OpenCR) can't be split across
// the worker boundary this way -- but that combination doesn't arise
// (WebSerial ports aren't transferable to a SharedWorker anyway).

import { getCodec } from '../../transport/src/codecs.js';

export class HardwareBridgeClient {
  constructor(workerUrl, { codec = getCodec() } = {}) {
    this._worker = new SharedWorker(workerUrl, { type: 'module', name: 'ros-chromium-hardware-bridge' });
    this._port = this._worker.port;
    this._codec = codec;
    this._connected = false;
    this._connectResolve = null;
    this._connectReject = null;

    this._messageHandlers = [];
    this._disconnectHandlers = [];
    this._heartbeatSentHandlers = [];
    this._heartbeatGapHandlers = [];
    this._heartbeatErrorHandlers = [];
    this._sendErrorHandlers = [];

    this._port.onmessage = (e) => this._handleMessage(e.data);
  }

  _handleMessage(msg) {
    switch (msg.type) {
      case 'status':
        this._connected = msg.connected;
        if (msg.connected) {
          this._connectResolve?.();
          this._connectResolve = this._connectReject = null;
        } else {
          for (const cb of this._disconnectHandlers) cb();
        }
        break;
      case 'connect-error':
        this._connectReject?.(new Error(msg.message));
        this._connectResolve = this._connectReject = null;
        break;
      case 'message':
        for (const cb of this._messageHandlers) cb(msg.msg);
        break;
      case 'heartbeat-sent':
        for (const cb of this._heartbeatSentHandlers) cb();
        break;
      case 'heartbeat-gap':
        for (const cb of this._heartbeatGapHandlers) cb(msg.gap);
        break;
      case 'heartbeat-error':
        for (const cb of this._heartbeatErrorHandlers) cb(new Error(msg.message));
        break;
      case 'send-error':
        for (const cb of this._sendErrorHandlers) cb(new Error(msg.message));
        break;
    }
  }

  connect() {
    return new Promise((resolve, reject) => {
      this._connectResolve = resolve;
      this._connectReject = reject;
      this._port.postMessage({ type: 'connect' });
    });
  }

  // Fire-and-forget: posting to the worker doesn't wait for the real
  // transport's send to finish, so this resolves as soon as the message is
  // queued, not once the frame has actually left the worker. A failure
  // shows up asynchronously via onSendError instead of rejecting this
  // call's promise — correlating each send to its outcome would mean
  // round-tripping every single heartbeat, which isn't worth it here.
  async send(frame) {
    this._port.postMessage({ type: 'send', frame });
  }

  // Encode one command to wire bytes with this client's codec, so callers
  // (createDriveDevice, OdometryNode) don't import a protocol module.
  encode(spec) {
    return this._codec.encode(spec);
  }

  onMessage(cb) {
    this._messageHandlers.push(cb);
  }

  onDisconnect(cb) {
    this._disconnectHandlers.push(cb);
  }

  onHeartbeatSent(cb) {
    this._heartbeatSentHandlers.push(cb);
  }

  onHeartbeatGap(cb) {
    this._heartbeatGapHandlers.push(cb);
  }

  onHeartbeatError(cb) {
    this._heartbeatErrorHandlers.push(cb);
  }

  onSendError(cb) {
    this._sendErrorHandlers.push(cb);
  }
}
