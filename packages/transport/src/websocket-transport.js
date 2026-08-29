// WebSocketTransport — the first real HardwareTransport implementation.
// Uses the global `WebSocket` (available in every browser and, since
// Node 22, in Node itself) so the exact same class runs unmodified in a
// real Chromium tab and in a Node.js test client. This is what stands in
// for `WebSerialTransport` until a target board exists (plan.md Phase 2).

import { FrameDecoder } from './frame.js';

export class WebSocketTransport {
  constructor(url) {
    this._url = url;
    this._ws = null;
    this._decoder = new FrameDecoder();
    this._frameHandlers = [];
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
        for (const frame of this._decoder.push(bytes)) {
          for (const cb of this._frameHandlers) cb(frame);
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

  onFrame(cb) {
    this._frameHandlers.push(cb);
  }

  onDisconnect(cb) {
    this._disconnectHandlers.push(cb);
  }
}
