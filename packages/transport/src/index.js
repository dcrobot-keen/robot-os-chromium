// HardwareTransport — the interface every concrete transport implements
// (see the architecture doc, Layer 02). Documented as a shape here rather
// than a TS interface for now; see plan.md for the note on migrating this
// package to TypeScript once the shape stabilizes.
//
// connect(): Promise<void>
// send(frame: Uint8Array): Promise<void>
// encode(spec): Uint8Array                — one command -> wire bytes, via this transport's codec
// onMessage(cb: (msg) => void): void      — normalized reply {type,key,values,...} (see codecs.js)
// onDisconnect(cb: () => void): void
// onRaw?(cb: (bytes: Uint8Array) => void): void  — optional: undecoded bytes, for relays
// close?(): void                                 — optional: deliberate clean shutdown
//
// The wire protocol is selected by a manifest's `transport.kind` via
// codecs.js (getCodec). The only codec today is the Roboteq ASCII line
// protocol of the Former 2.0 base + the simulator (former-motor-protocol.md);
// roboteq.js holds it: encodeCommand + the `cmd` builders + RoboteqDecoder.
// A transport takes an optional { codec } and exposes encode(spec), so
// nothing above HardwareTransport imports a codec directly — see roadmap.md
// "세 타깃 동시 진행".
//
// Implemented:
//   - WebSocketTransport — global WebSocket; browser tab or Node client.
//     Used to test against the firmware simulator.
//   - WebSerialTransport — navigator.serial @ 115200 to /dev/ttyMOTOR, for
//     the real Roboteq controller on the robot. Chromium only.
//   - startHeartbeat — the "!B 3 1" keepalive loop (heartbeat.js).

export * from './roboteq.js';
export * from './dynamixel-protocol2.js';
export * from './turtlebot3-opencr.js';
export * from './lidar-lds.js';
export * from './codecs.js';
export * from './websocket-transport.js';
export * from './web-serial-transport.js';
export * from './heartbeat.js';
