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
// Implemented:
//   - WebSocketTransport — global WebSocket; browser tab or Node client.
//     Used to test against the firmware simulator.
//   - WebSerialTransport — navigator.serial @ 115200 to /dev/ttyMOTOR, for
//     the real Roboteq controller on the robot. Chromium only.
//   - startHeartbeat — the "!B 3 1" keepalive loop (heartbeat.js).

export * from './roboteq.js';
export * from './websocket-transport.js';
export * from './web-serial-transport.js';
export * from './heartbeat.js';
