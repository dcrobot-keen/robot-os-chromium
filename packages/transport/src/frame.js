// Wire protocol codec: SOF | LEN | CMD | PAYLOAD | CRC16 | EOF
// (see research.md, "Wire protocol — framed binary over USB-CDC").
//
// NOTE: duplicated in firmware/sim/src/frame.js on purpose — see the note
// in commands.js. Keep both copies byte-for-byte identical.

export const SOF = 0x7e;
export const EOF_BYTE = 0x7f;

// CRC-16/CCITT-FALSE (poly 0x1021, init 0xffff) over CMD + PAYLOAD.
export function crc16(bytes) {
  let crc = 0xffff;
  for (const b of bytes) {
    crc ^= b << 8;
    for (let i = 0; i < 8; i++) {
      crc = crc & 0x8000 ? ((crc << 1) ^ 0x1021) & 0xffff : (crc << 1) & 0xffff;
    }
  }
  return crc;
}

export function encodeFrame(cmd, payload = new Uint8Array()) {
  const body = [cmd, ...payload];
  const crc = crc16(body);
  const frame = new Uint8Array(payload.length + 6);
  frame[0] = SOF;
  frame[1] = payload.length;
  frame[2] = cmd;
  frame.set(payload, 3);
  frame[3 + payload.length] = (crc >> 8) & 0xff;
  frame[4 + payload.length] = crc & 0xff;
  frame[5 + payload.length] = EOF_BYTE;
  return frame;
}

// Streaming decoder: feed raw bytes in as they arrive, get complete,
// CRC-verified frames out. Resyncs on the next SOF if a frame is malformed.
export class FrameDecoder {
  constructor() {
    this._buf = [];
  }

  push(bytes) {
    for (const b of bytes) this._buf.push(b);
    const frames = [];
    for (;;) {
      const sofIdx = this._buf.indexOf(SOF);
      if (sofIdx === -1) {
        this._buf = [];
        break;
      }
      if (sofIdx > 0) this._buf.splice(0, sofIdx);
      if (this._buf.length < 2) break; // need LEN byte
      const len = this._buf[1];
      const frameLen = 6 + len;
      if (this._buf.length < frameLen) break; // wait for more bytes

      const frame = this._buf.splice(0, frameLen);
      if (frame[frameLen - 1] !== EOF_BYTE) continue; // malformed, resync
      const cmd = frame[2];
      const payload = frame.slice(3, 3 + len);
      const gotCrc = (frame[3 + len] << 8) | frame[4 + len];
      if (crc16([cmd, ...payload]) !== gotCrc) continue; // corrupt, drop

      frames.push({ cmd, payload: Uint8Array.from(payload) });
    }
    return frames;
  }
}
