// RtcHostBridge — the host side of a remote session. Runs on the machine
// connected to the robot (the Former's own PC), or any machine that can
// reach the firmware simulator. For each operator that shows up in the room
// it answers the WebRTC offer, then acts as a transparent byte pipe — both
// directions, undecoded — between that operator's data channel and a
// HardwareTransport to the controller:
//
//   operator data channel  <--raw bytes-->  controller
//
// The transport comes from the `makeTransport` factory passed in, so the
// same bridge fronts a WebSocketTransport (the sim) or a WebSerialTransport
// (the real Roboteq on /dev/ttyMOTOR) unchanged.
//
// What it deliberately does NOT do:
//   - It never sends the keepalive. The operator owns it (see
//     rtc-transport.js). If the operator drops, the Roboteq serial
//     watchdog stops the motors on its own ~1s later.
//   - It never issues ESTOP or parses operator traffic — relay only
//     (transport.onRaw, not onMessage).
//   - The one exception: right after connecting it sends `initCommands`
//     (from the manifest — ^ECHOF 1 / !R 2 / !AC / !DC for a real Roboteq,
//     empty for the sim). That's controller bring-up, not driving.
//
// One transport per operator session — the controller re-arms out of its
// post-watchdog / post-ESTOP state on a *new* connection (a !MG also does
// it), so "operator connected" <-> "transport open" keeps connect -> drive
// -> disconnect -> reconnect correct. NOTE: a real serial port is
// exclusive, so 2+ simultaneous operators would need the bridge to hold ONE
// shared transport (like the SharedWorker does for tabs) — out of scope
// while Phase 5 is single-operator (plan.md).
//
// The per-operator firmware transport comes from the injected makeTransport
// factory; init commands are encoded with that transport's own codec
// (firmware.encode), so this file is not tied to a wire protocol.

const DEFAULT_RTC_CONFIG = {
  iceServers: [{ urls: 'stun:stun.l.google.com:19302' }],
};

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

export class RtcHostBridge {
  // makeTransport: () => HardwareTransport (a fresh one per operator session)
  // initCommands: string[] — Roboteq lines sent once after each connect
  constructor(signalingClient, makeTransport, { rtcConfig = DEFAULT_RTC_CONFIG, initCommands = [], onEvent } = {}) {
    this._sig = signalingClient;
    this._makeTransport = makeTransport;
    this._initCommands = initCommands;
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
      // Fresh transport to the controller for this operator session — this
      // is the "connection" the watchdog re-arms on.
      const firmware = this._makeTransport();
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
        // Controller bring-up (Roboteq: ^ECHOF 1 / !R 2 / !AC / !DC). Spaced
        // like the reference ROS driver; skipped when initCommands is empty.
        for (const line of this._initCommands) {
          await firmware.send(firmware.encode(line));
          await sleep(100);
        }
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
