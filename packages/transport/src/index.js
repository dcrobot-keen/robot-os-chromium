// HardwareTransport — the interface every concrete transport implements
// (see the architecture doc, Layer 02). Documented as a shape here rather
// than a TS interface for now; see plan.md for the note on migrating this
// package to TypeScript once the shape stabilizes.
//
// connect(): Promise<void>
// send(frame: Uint8Array): Promise<void>
// onFrame(cb: (frame: Uint8Array) => void): void
// onDisconnect(cb: () => void): void
//
// Implemented so far:
//   - none exported yet. The simplest prototype (scripts/prototype-client.mjs
//     at the repo root) talks raw TCP directly with frame.js, deliberately
//     skipping this interface to validate the protocol/watchdog first.
//
// TODO, in order (plan.md Phase 1 → 4):
//   - WebSerialTransport   (needs real hardware to test against)
//   - WebSocketTransport   (browser-facing equivalent of the TCP prototype)
//   - WebUSBTransport / WebHIDTransport / WebBluetoothTransport (as needed)

export * from './frame.js';
export * from './commands.js';
