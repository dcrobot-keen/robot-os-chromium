// HardwareTransport — the interface every concrete transport implements
// (see the architecture doc, Layer 02). Documented as a shape here rather
// than a TS interface for now; see plan.md for the note on migrating this
// package to TypeScript once the shape stabilizes.
//
// connect(): Promise<void>
// send(frame: Uint8Array): Promise<void>
// onFrame(cb: (frame: { cmd: number, payload: Uint8Array }) => void): void
// onDisconnect(cb: () => void): void
//
// Implemented so far:
//   - WebSocketTransport — uses the global WebSocket, so it runs unchanged
//     in a real browser tab and in a Node.js test client (Node 22+).
//
// TODO, in order (plan.md Phase 2 → 4):
//   - WebSerialTransport   (needs a real board to test against)
//   - WebUSBTransport / WebHIDTransport / WebBluetoothTransport (as needed)

export * from './frame.js';
export * from './commands.js';
export * from './websocket-transport.js';
