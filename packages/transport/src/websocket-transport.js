// WebSocketTransport — the HardwareTransport implementation used for
// testing against the firmware simulator (a real board will get a
// WebSerialTransport with the same shape). Uses the global `WebSocket`
// (browser, and Node 22+) so the exact same class runs unmodified in a
// Chromium tab and in a Node.js test client.
//
// The payload on the wire is now the Roboteq ASCII line protocol
// (former-motor-protocol.md): send() takes the bytes of a command line
// (build them with encodeCommand from roboteq.js); onMessage() delivers
// parsed replies ({type:'ack'|'reply'|'line', ...}); onRaw() delivers the
// undecoded bytes, for a pass-through relay that must not parse (the
// Phase 5 RtcHostBridge).

import { RoboteqDecoder } from './roboteq.js';

export class WebSocketTransport {
  constructor(url) {
    this._url = url;
    this._ws = null;
    this._decoder = new RoboteqDecoder();
    this._messageHandlers = [];
    this._rawHandlers = [];
    this._disconnectHandlers = [];
  }

  connect() {
    return new Promise((resolve, reject) => {
      const ws = new WebSocket(this._url);
      ws.binaryType = 'arraybuffer';

      ws.onopen = () => {
        this._ws = ws;
        resolve();
      };
      ws.onerror = (err) => reject(err);
      ws.onmessage = (event) => {
        const bytes = new Uint8Array(event.data);
        for (const cb of this._rawHandlers) cb(bytes);
        for (const msg of this._decoder.push(bytes)) {
          for (const cb of this._messageHandlers) cb(msg);
        }
      };
      ws.onclose = () => {
        for (const cb of this._disconnectHandlers) cb();
      };
    });
  }

  async send(frame) {
    this._ws.send(frame);
  }

  // Optional part of the interface: a clean, deliberate close (sends the WS
  // close frame). The prototype client still avoids this on purpose to
  // simulate a crash (see its comment); the Phase 5 host bridge uses it to
  // end one operator's firmware session tidily so the next one re-arms.
  close() {
    this._ws?.close();
  }

  onMessage(cb) {
    this._messageHandlers.push(cb);
  }

  onRaw(cb) {
    this._rawHandlers.push(cb);
  }

  onDisconnect(cb) {
    this._disconnectHandlers.push(cb);
  }
}
