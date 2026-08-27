// hardware-bridge-worker — runs as a SharedWorker, not imported as a normal
// module. This is the single owner of the real hardware connection across
// every tab of the same origin (architecture doc, Layer 05: "a SharedWorker
// owning the actual hardware connection so multiple tabs don't fight over
// it"; plan.md Phase 4).
//
// Loaded from a page with: new SharedWorker(url, { type: 'module' })
//
// Protocol between a tab's port and this worker (all messages are plain
// objects, sent with postMessage — Uint8Array frames survive structured
// clone fine, no manual serialization needed):
//   tab -> worker   { type: 'connect' }
//   tab -> worker   { type: 'send', frame: Uint8Array }
//   worker -> tab   { type: 'status', connected: boolean }        (broadcast)
//   worker -> tab   { type: 'connect-error', message }            (to the requester only)
//   worker -> tab   { type: 'send-error', message }               (to the requester only)
//   worker -> tab   { type: 'frame', cmd, payload }                (broadcast — includes heartbeat echoes)
//   worker -> tab   { type: 'heartbeat-sent' }                     (broadcast)
//   worker -> tab   { type: 'heartbeat-gap', gap }                 (broadcast)
//   worker -> tab   { type: 'heartbeat-error', message }           (broadcast)

import { WebSocketTransport } from '../../transport/src/websocket-transport.js';
import { startHeartbeat } from '../../transport/src/heartbeat.js';

const WS_URL = 'ws://127.0.0.1:8765';

const ports = new Set();
const transport = new WebSocketTransport(WS_URL);
let connected = false;
let connectingPromise = null;

function broadcast(msg) {
  for (const port of ports) port.postMessage(msg);
}

transport.onFrame((frame) => broadcast({ type: 'frame', cmd: frame.cmd, payload: frame.payload }));
transport.onDisconnect(() => {
  connected = false;
  broadcast({ type: 'status', connected: false });
});

// Idempotent: the first tab to ask triggers the real connect + heartbeat
// start; every tab after that (including ones that open later) just gets
// told the current status.
function ensureConnected() {
  if (connected) return Promise.resolve();
  if (!connectingPromise) {
    connectingPromise = transport
      .connect()
      .then(() => {
        connected = true;
        broadcast({ type: 'status', connected: true });
        startHeartbeat(transport, {
          onGap: (gap) => broadcast({ type: 'heartbeat-gap', gap }),
          onSend: () => broadcast({ type: 'heartbeat-sent' }),
          onSendError: (err) => broadcast({ type: 'heartbeat-error', message: err.message || String(err) }),
        });
      })
      .finally(() => {
        connectingPromise = null;
      });
  }
  return connectingPromise;
}

self.onconnect = (event) => {
  const port = event.ports[0];
  ports.add(port);

  port.onmessage = async (e) => {
    const msg = e.data;
    if (msg.type === 'connect') {
      try {
        await ensureConnected();
        port.postMessage({ type: 'status', connected: true });
      } catch (err) {
        port.postMessage({ type: 'connect-error', message: err.message || String(err) });
      }
    } else if (msg.type === 'send') {
      try {
        await transport.send(msg.frame);
      } catch (err) {
        port.postMessage({ type: 'send-error', message: err.message || String(err) });
      }
    }
  };

  // A brand-new port needs to know the current state immediately — it
  // won't have seen whatever status broadcast happened before it existed.
  port.postMessage({ type: 'status', connected });
};
