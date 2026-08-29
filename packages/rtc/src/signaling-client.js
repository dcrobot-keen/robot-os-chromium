// SignalingClient — the peer-side half of apps/signaling-server. Wraps one
// WebSocket to the signaling server and turns its JSON messages into
// callbacks. Uses the global `WebSocket` (browser, and Node 22+) so the
// exact same class is used by the browser dashboard and by the Node smoke
// test (scripts/signaling-smoke.mjs).
//
// It only handles the *rendezvous*: who is in the room, and relaying the
// opaque `data` blobs that RtcTransport / RtcHostBridge use to carry SDP
// and ICE. It has no idea what WebRTC is.

export class SignalingClient {
  constructor(url, { role, robot, manifest = null } = {}) {
    this._url = url;
    this._role = role;
    this._robot = robot;
    this._manifest = manifest;
    this._ws = null;
    this.peerId = null;

    this._readyResolve = null;
    this._readyReject = null;
    this._peerJoinedHandlers = [];
    this._peerLeftHandlers = [];
    this._signalHandlers = [];
    this._closeHandlers = [];
  }

  // Resolves with { peerId, peers: [{peerId, role}] } once the server has us
  // in the room — `peers` is who was already there, which tells the caller
  // whether to kick off the WebRTC offer now or wait for peer-joined.
  connect() {
    return new Promise((resolve, reject) => {
      this._readyResolve = resolve;
      this._readyReject = reject;

      const ws = new WebSocket(this._url);
      this._ws = ws;

      ws.onopen = () => {
        ws.send(JSON.stringify({
          type: 'hello',
          role: this._role,
          robot: this._robot,
          ...(this._manifest ? { manifest: this._manifest } : {}),
        }));
      };
      ws.onerror = (err) => this._readyReject?.(err instanceof Error ? err : new Error('signaling socket error'));
      ws.onclose = () => {
        for (const cb of this._closeHandlers) cb();
      };
      ws.onmessage = (event) => this._handle(JSON.parse(typeof event.data === 'string' ? event.data : event.data.toString()));
    });
  }

  _handle(msg) {
    switch (msg.type) {
      case 'ready':
        this.peerId = msg.peerId;
        this._readyResolve?.({ peerId: msg.peerId, peers: msg.peers || [] });
        this._readyResolve = this._readyReject = null;
        break;
      case 'peer-joined':
        for (const cb of this._peerJoinedHandlers) cb({ peerId: msg.peerId, role: msg.role });
        break;
      case 'peer-left':
        for (const cb of this._peerLeftHandlers) cb({ peerId: msg.peerId, role: msg.role });
        break;
      case 'signal':
        for (const cb of this._signalHandlers) cb({ from: msg.from, data: msg.data });
        break;
      case 'error':
        // If we're still waiting on `ready`, a server error means the
        // rendezvous failed (e.g. "robot already has a host") — surface it.
        this._readyReject?.(new Error(msg.message));
        this._readyResolve = this._readyReject = null;
        break;
    }
  }

  sendSignal(data, to = null) {
    this._ws.send(JSON.stringify({ type: 'signal', ...(to ? { to } : {}), data }));
  }

  onPeerJoined(cb) { this._peerJoinedHandlers.push(cb); }
  onPeerLeft(cb) { this._peerLeftHandlers.push(cb); }
  onSignal(cb) { this._signalHandlers.push(cb); }
  onClose(cb) { this._closeHandlers.push(cb); }

  close() {
    this._ws?.close();
  }
}
