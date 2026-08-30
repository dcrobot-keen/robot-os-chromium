// RtcTransport — the operator side of a remote session. Implements the same
// connect()/send()/onMessage()/onDisconnect() shape as WebSocketTransport
// (see packages/transport/src/index.js), so the dashboard drives a robot on
// another machine with exactly the code it uses to drive one plugged into
// this machine — only the transport constructor changes. Same trick as
// HardwareBridgeClient in Phase 4, one hop further out.
//
// The operator is the active party: it creates the RTCPeerConnection and
// the data channel and sends the offer. The host just answers (see
// rtc-host-bridge.js). Roles are fixed, so there's no perfect-negotiation
// dance here.
//
// The heartbeat rides this transport. That is deliberate: if the operator's
// machine freezes or its network drops, the "!B 3 1" keepalive stops
// arriving at the host, the host forwards nothing, and the Roboteq serial
// watchdog zeroes the motors ~1s later — the same guarantee as a yanked
// cable, now across a WebRTC link. The host never keepalives on the
// operator's behalf.

import { getCodec } from '../../transport/src/codecs.js';

const DEFAULT_RTC_CONFIG = {
  // A public STUN server covers same-LAN / simple-NAT cases. Cross-NAT
  // needs a TURN server; that (and the LAN WebSocket-relay fallback) is
  // still an open design item — see plan.md "아직 정하지 않은 것".
  iceServers: [{ urls: 'stun:stun.l.google.com:19302' }],
};

export class RtcTransport {
  constructor(signalingClient, { rtcConfig = DEFAULT_RTC_CONFIG, codec = getCodec() } = {}) {
    this._sig = signalingClient;
    this._rtcConfig = rtcConfig;
    this._pc = null;
    this._dc = null;
    this._hostPeerId = null;
    this._codec = codec;
    this._decoder = new codec.Decoder();
    this._messageHandlers = [];
    this._disconnectHandlers = [];
    this._disconnected = false;
  }

  encode(spec) {
    return this._codec.encode(spec);
  }

  async connect() {
    const { peers } = await this._sig.connect();
    this._hostPeerId = peers.find((p) => p.role === 'host')?.peerId ?? null;
    this._pc = new RTCPeerConnection(this._rtcConfig);

    this._pc.onicecandidate = (e) => {
      if (e.candidate) this._sig.sendSignal({ candidate: e.candidate }, this._hostPeerId);
    };
    this._pc.onconnectionstatechange = () => {
      const st = this._pc.connectionState;
      if (st === 'failed' || st === 'disconnected' || st === 'closed') this._fireDisconnect();
    };

    this._sig.onSignal(async ({ data }) => {
      if (data.sdp) {
        await this._pc.setRemoteDescription(data.sdp);
      } else if (data.candidate) {
        try {
          await this._pc.addIceCandidate(data.candidate);
        } catch {
          // a stray candidate before the remote description is set — WebRTC
          // buffers these itself in modern browsers; ignore the throw
        }
      }
    });

    const dcOpen = new Promise((resolve, reject) => {
      this._dc = this._pc.createDataChannel('robot', { ordered: true });
      this._dc.binaryType = 'arraybuffer';
      this._dc.onopen = () => resolve();
      this._dc.onclose = () => this._fireDisconnect();
      this._dc.onerror = () => reject(new Error('data channel error'));
      this._dc.onmessage = (event) => {
        const bytes = new Uint8Array(event.data);
        for (const msg of this._decoder.push(bytes)) {
          for (const cb of this._messageHandlers) cb(msg);
        }
      };
    });

    // If the host is already in the room, offer now; otherwise wait for it.
    if (this._hostPeerId !== null) {
      await this._offer();
    } else {
      this._sig.onPeerJoined(({ peerId, role }) => {
        if (role === 'host' && this._hostPeerId === null) {
          this._hostPeerId = peerId;
          this._offer();
        }
      });
    }

    await dcOpen;
  }

  async _offer() {
    const offer = await this._pc.createOffer();
    await this._pc.setLocalDescription(offer);
    this._sig.sendSignal({ sdp: this._pc.localDescription }, this._hostPeerId);
  }

  _fireDisconnect() {
    if (this._disconnected) return;
    this._disconnected = true;
    for (const cb of this._disconnectHandlers) cb();
  }

  async send(frame) {
    // Throws if the channel isn't open — same as WebSocketTransport.send
    // throwing on a dead socket; startHeartbeat reports it via onSendError.
    this._dc.send(frame);
  }

  onMessage(cb) { this._messageHandlers.push(cb); }
  onDisconnect(cb) { this._disconnectHandlers.push(cb); }

  close() {
    this._dc?.close();
    this._pc?.close();
    this._sig.close();
  }
}
