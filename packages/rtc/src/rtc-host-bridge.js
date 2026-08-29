// RtcHostBridge — the host side of a remote session. Runs on the machine
// that is physically connected to the robot (here: the machine that can
// reach the firmware simulator). For each operator that shows up in the
// room it answers the WebRTC offer, then acts as a near-transparent byte
// pipe between that operator's data channel and a WebSocket to the
// firmware:
//
//   operator data channel  <--raw bytes-->  firmware
//
// What it deliberately does NOT do:
//   - It never sends the keepalive. The operator owns it (see
//     rtc-transport.js). If the operator drops, the Roboteq serial
//     watchdog stops the motors on its own ~1s later.
//   - It never issues ESTOP or parses traffic. The firmware is the
//     authority; this is just a relay (uses transport.onRaw, not onMessage).
//
// One firmware WebSocket per operator session. The firmware only re-arms
// out of its post-watchdog / post-ESTOP safe state on a *new* connection
// (a !MG from the operator also does it), so mapping "an operator is
// connected" to "a WebSocket to the firmware is open" keeps connect ->
// drive -> disconnect -> reconnect correct.
// Whether the real on-robot host should behave the same way — or expose an
// explicit re-arm — is an open item (plan.md Phase 5).

import { WebSocketTransport } from '../../transport/src/websocket-transport.js';

const DEFAULT_RTC_CONFIG = {
  iceServers: [{ urls: 'stun:stun.l.google.com:19302' }],
};

export class RtcHostBridge {
  constructor(signalingClient, firmwareUrl, { rtcConfig = DEFAULT_RTC_CONFIG, onEvent } = {}) {
    this._sig = signalingClient;
    this._firmwareUrl = firmwareUrl;
    this._rtcConfig = rtcConfig;
    this._onEvent = onEvent || (() => {});
    // operator peerId -> { pc, dc, firmware, bytesToFirmware, bytesToOperator }
    this._sessions = new Map();
  }

  async start() {
    const { peers } = await this._sig.connect();

    this._sig.onSignal(({ from, data }) => this._onSignal(from, data));
    this._sig.onPeerJoined(({ peerId, role }) => {
      if (role === 'operator') this._openSession(peerId);
    });
    this._sig.onPeerLeft(({ peerId }) => this._closeSession(peerId, 'operator left'));

    for (const p of peers) {
      if (p.role === 'operator') this._openSession(p.peerId);
    }
    this._onEvent({ type: 'started', peerId: this._sig.peerId });
  }

  _openSession(operatorId) {
    if (this._sessions.has(operatorId)) return;

    const pc = new RTCPeerConnection(this._rtcConfig);
    const session = { pc, dc: null, firmware: null, bytesToFirmware: 0, bytesToOperator: 0 };
    this._sessions.set(operatorId, session);

    pc.onicecandidate = (e) => {
      if (e.candidate) this._sig.sendSignal({ candidate: e.candidate }, operatorId);
    };
    pc.onconnectionstatechange = () => {
      const st = pc.connectionState;
      if (st === 'failed' || st === 'disconnected' || st === 'closed') {
        this._closeSession(operatorId, `peer connection ${st}`);
      }
    };
    pc.ondatachannel = (e) => this._wireDataChannel(operatorId, session, e.channel);

    this._onEvent({ type: 'operator-joined', operatorId });
  }

  _wireDataChannel(operatorId, session, dc) {
    session.dc = dc;
    dc.binaryType = 'arraybuffer';

    dc.onopen = async () => {
      // Fresh WebSocket to the firmware for this operator session — this is
      // the "connection" the firmware watchdog re-arms on.
      const firmware = new WebSocketTransport(this._firmwareUrl);
      session.firmware = firmware;
      firmware.onRaw((bytes) => {
        if (session.dc?.readyState === 'open') {
          session.dc.send(bytes);
          session.bytesToOperator += bytes.length;
        }
      });
      firmware.onDisconnect(() => this._closeSession(operatorId, 'firmware connection closed'));
      try {
        await firmware.connect();
        this._onEvent({ type: 'firmware-connected', operatorId });
      } catch (err) {
        this._onEvent({ type: 'firmware-error', operatorId, message: err.message || String(err) });
        this._closeSession(operatorId, 'firmware connect failed');
      }
    };

    dc.onclose = () => this._closeSession(operatorId, 'data channel closed');
    dc.onerror = () => this._closeSession(operatorId, 'data channel error');
    dc.onmessage = (event) => {
      const bytes = new Uint8Array(event.data);
      session.bytesToFirmware += bytes.length;
      // Forward raw — the firmware's own decoder is the authority on what's
      // a valid command line. The bridge does not parse operator traffic.
      session.firmware?.send(bytes).catch((err) => {
        this._onEvent({ type: 'firmware-send-error', operatorId, message: err.message || String(err) });
      });
    };
  }

  async _onSignal(operatorId, data) {
    const session = this._sessions.get(operatorId);
    if (!session) return;
    const { pc } = session;

    if (data.sdp) {
      await pc.setRemoteDescription(data.sdp);
      if (data.sdp.type === 'offer') {
        const answer = await pc.createAnswer();
        await pc.setLocalDescription(answer);
        this._sig.sendSignal({ sdp: pc.localDescription }, operatorId);
      }
    } else if (data.candidate) {
      try {
        await pc.addIceCandidate(data.candidate);
      } catch {
        // candidate arrived before the remote description; browsers buffer
        // these internally, so a throw here is safe to ignore
      }
    }
  }

  _closeSession(operatorId, reason) {
    const session = this._sessions.get(operatorId);
    if (!session) return;
    this._sessions.delete(operatorId);

    // Best-effort clean close of the firmware socket so the next operator
    // session starts from a re-armed firmware.
    try { session.firmware?.close(); } catch { /* already gone */ }
    try { session.dc?.close(); } catch { /* already gone */ }
    try { session.pc.close(); } catch { /* already gone */ }

    this._onEvent({ type: 'operator-left', operatorId, reason });
  }

  stop() {
    for (const operatorId of [...this._sessions.keys()]) this._closeSession(operatorId, 'bridge stopped');
    this._sig.close();
  }
}
