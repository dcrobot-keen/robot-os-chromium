// WebSerialTransport — HardwareTransport over navigator.serial, for talking
// to the real Roboteq controller on the robot's own machine
// (former-motor-protocol.md: /dev/ttyMOTOR @ 115200 8N1). Same
// connect()/send()/onMessage()/onRaw()/onDisconnect()/close() shape as
// WebSocketTransport, so RtcHostBridge / createDriveDevice don't care which
// one they're handed.
//
// Chromium only (navigator.serial), and only in a secure context
// (localhost counts). Can't run in Node — node --check here, real
// verification is hands-on on the robot.
//
// connect() shows the browser's port picker the first time. To avoid that
// on every boot, grant the port once and/or install the enterprise policy
// SerialAllowUsbDevicesForUrls for the dashboard origin + the FTDI
// vendor/product (0x0403 / 0x6001 for the Former's adapter). When exactly
// one port has already been granted, connect() reuses it without
// prompting.

import { RoboteqDecoder } from './roboteq.js';

export class WebSerialTransport {
  // filters: [{ usbVendorId, usbProductId }] passed to requestPort() so the
  // picker is pre-narrowed. Former's motor adapter: 0x0403 / 0x6001.
  constructor({ baudRate = 115200, filters = [] } = {}) {
    this._baudRate = baudRate;
    this._filters = filters;
    this._port = null;
    this._reader = null;
    this._writer = null;
    this._decoder = new RoboteqDecoder();
    this._messageHandlers = [];
    this._rawHandlers = [];
    this._disconnectHandlers = [];
    this._closing = false;
    this._disconnected = false;
  }

  async connect() {
    if (typeof navigator === 'undefined' || !('serial' in navigator)) {
      throw new Error('WebSerial unavailable — needs Chromium in a secure context');
    }

    // Reuse an already-granted port if there's exactly one; otherwise prompt.
    const granted = await navigator.serial.getPorts();
    if (granted.length === 1) {
      this._port = granted[0];
    } else {
      this._port = await navigator.serial.requestPort(
        this._filters.length ? { filters: this._filters } : undefined,
      );
    }

    await this._port.open({ baudRate: this._baudRate }); // 8 data bits / no parity / 1 stop bit are the defaults

    this._writer = this._port.writable.getWriter();
    this._readLoop(); // runs until the port closes or errors

    // Physical unplug (distinct from a clean close()).
    this._port.addEventListener('disconnect', () => this._fireDisconnect());
  }

  async _readLoop() {
    this._reader = this._port.readable.getReader();
    try {
      for (;;) {
        const { value, done } = await this._reader.read();
        if (done) break;
        if (!value || value.length === 0) continue;
        for (const cb of this._rawHandlers) cb(value);
        for (const msg of this._decoder.push(value)) {
          for (const cb of this._messageHandlers) cb(msg);
        }
      }
    } catch {
      // reader throws when the port is lost or cancelled — treated as a disconnect below
    } finally {
      try { this._reader.releaseLock(); } catch { /* already released */ }
      if (!this._closing) this._fireDisconnect();
    }
  }

  async send(frame) {
    await this._writer.write(frame instanceof Uint8Array ? frame : new Uint8Array(frame));
  }

  async close() {
    this._closing = true;
    try { await this._reader?.cancel(); } catch { /* ignore */ }
    try { await this._writer?.close(); } catch { /* ignore */ }
    try { this._writer?.releaseLock(); } catch { /* ignore */ }
    try { await this._port?.close(); } catch { /* ignore */ }
  }

  onMessage(cb) { this._messageHandlers.push(cb); }
  onRaw(cb) { this._rawHandlers.push(cb); }
  onDisconnect(cb) { this._disconnectHandlers.push(cb); }

  _fireDisconnect() {
    if (this._disconnected) return;
    this._disconnected = true;
    for (const cb of this._disconnectHandlers) cb();
  }
}
