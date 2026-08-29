// HardwareTransport — the interface every concrete transport implements
// (see the architecture doc, Layer 02). Documented as a shape here rather
// than a TS interface for now; see plan.md for the note on migrating this
// package to TypeScript once the shape stabilizes.
//
// connect(): Promise<void>
// send(frame: Uint8Array): Promise<void>
// onMessage(cb: (msg) => void): void      — parsed Roboteq reply (see roboteq.js)
// onDisconnect(cb: () => void): void
// onRaw?(cb: (bytes: Uint8Array) => void): void  — optional: undecoded bytes, for relays
// close?(): void                                 — optional: deliberate clean shutdown
//
// The wire protocol is the Roboteq ASCII line protocol of the Former 2.0
// base (former-motor-protocol.md). roboteq.js holds the codec: encodeCommand
// + the `cmd` builders + RoboteqDecoder.
//
// Implemented so far:
//   - WebSocketTransport — global WebSocket; runs in a browser tab and in a
//     Node.js test client. Stands in for WebSerialTransport until the real
//     RS232 link to the Roboteq controller is wired up.
//   - startHeartbeat — the "!B 3 1" keepalive loop (heartbeat.js).
//
// TODO:
//   - WebSerialTransport — navigator.serial @ 115200 to /dev/ttyMOTOR
//     (former-motor-protocol.md, "web/packages/transport → Roboteq 코덱").

export * from './roboteq.js';
export * from './websocket-transport.js';
export * from './heartbeat.js';
