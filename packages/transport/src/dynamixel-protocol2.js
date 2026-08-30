// DYNAMIXEL Protocol 2.0 — the wire format the TurtleBot3's OpenCR board
// speaks (it presents itself as one big DYNAMIXEL device, ID 200, with a
// control table; turtlebot3_node drives it over this). Protocol-standard,
// not TB3-specific — turtlebot3-opencr.js is the thin layer that maps our
// HardwareTransport calls onto control-table reads/writes.
//
// Packet (little-endian lengths, CRC over the whole packet up to the CRC):
//   FF FF FD 00 | ID | LEN_L LEN_H | INSTR | PARAMS... | CRC_L CRC_H
//   LEN counts INSTR + PARAMS + CRC (i.e. everything after LEN).
//
// Byte stuffing: a literal FF FF FD in PARAMS is sent as FF FF FD FD so it
// can't be mistaken for a header. Removed on receive.
//
// Reference vectors (ROBOTIS e-manual "Protocol 2.0"):
//   PING id 1            -> FF FF FD 00 01 03 00 01 19 4E
//   WRITE id 1 addr 116 val 512(LE 00 02 00 00)
//                        -> FF FF FD 00 01 09 00 03 74 00 00 02 00 00 CA 89
// dynamixel-smoke.mjs checks both.

const HEADER = [0xff, 0xff, 0xfd, 0x00];

export const INSTR = {
  PING: 0x01,
  READ: 0x02,
  WRITE: 0x03,
  REG_WRITE: 0x04,
  ACTION: 0x05,
  FACTORY_RESET: 0x06,
  REBOOT: 0x08,
  STATUS: 0x55,
};

// --- CRC-16 (poly 0x8005, MSB-first, init 0) — the ROBOTIS SDK table -----
const CRC_TABLE = (() => {
  const t = new Uint16Array(256);
  for (let i = 0; i < 256; i++) {
    let crc = i << 8;
    for (let j = 0; j < 8; j++) {
      crc = crc & 0x8000 ? (crc << 1) ^ 0x8005 : crc << 1;
    }
    t[i] = crc & 0xffff;
  }
  return t;
})();

export function crc16(bytes) {
  let crc = 0;
  for (let i = 0; i < bytes.length; i++) {
    const idx = ((crc >> 8) ^ bytes[i]) & 0xff;
    crc = ((crc << 8) ^ CRC_TABLE[idx]) & 0xffff;
  }
  return crc;
}

// --- little-endian value <-> bytes ------------------------------------
export function toLE(value, width) {
  const out = new Array(width);
  let v = value < 0 ? value + 2 ** (8 * width) : value; // two's complement
  for (let i = 0; i < width; i++) {
    out[i] = v & 0xff;
    v = Math.floor(v / 256);
  }
  return out;
}

export function fromLE(bytes, { signed = false } = {}) {
  let v = 0;
  for (let i = bytes.length - 1; i >= 0; i--) v = v * 256 + bytes[i];
  if (signed && bytes.length > 0 && bytes[bytes.length - 1] & 0x80) {
    v -= 2 ** (8 * bytes.length);
  }
  return v;
}

// --- byte stuffing ---------------------------------------------------
export function stuff(params) {
  const out = [];
  let run = 0; // count of trailing FF FF FD
  for (const b of params) {
    out.push(b);
    if (run === 0 && b === 0xff) run = 1;
    else if (run === 1 && b === 0xff) run = 2;
    else if (run === 2 && b === 0xfd) { out.push(0xfd); run = 0; }
    else run = b === 0xff ? 1 : 0;
  }
  return out;
}

export function unstuff(params) {
  const out = [];
  let run = 0;
  for (let i = 0; i < params.length; i++) {
    const b = params[i];
    if (run === 2 && b === 0xfd) {
      out.push(0xfd);
      // the very next byte (if 0xfd) is the stuffed one — skip it
      if (params[i + 1] === 0xfd) i++;
      run = 0;
      continue;
    }
    out.push(b);
    if (run < 2 && b === 0xff) run++;
    else if (b !== 0xff) run = 0;
  }
  return out;
}

// --- build an instruction packet -----------------------------------
export function buildInstruction({ id, instruction, params = [] }) {
  const stuffed = stuff(params);
  const len = stuffed.length + 3; // INSTR + PARAMS + CRC(2)
  const body = [...HEADER, id & 0xff, len & 0xff, (len >> 8) & 0xff, instruction & 0xff, ...stuffed];
  const crc = crc16(body);
  return new Uint8Array([...body, crc & 0xff, (crc >> 8) & 0xff]);
}

/** READ instruction: read `length` bytes starting at `address` from `id`. */
export function buildRead(id, address, length) {
  return buildInstruction({ id, instruction: INSTR.READ, params: [...toLE(address, 2), ...toLE(length, 2)] });
}

/** WRITE instruction: write `dataBytes` (array of 0..255) starting at `address`. */
export function buildWrite(id, address, dataBytes) {
  return buildInstruction({ id, instruction: INSTR.WRITE, params: [...toLE(address, 2), ...dataBytes] });
}

/** PING instruction. */
export function buildPing(id) {
  return buildInstruction({ id, instruction: INSTR.PING });
}

// --- streaming decoder for STATUS packets ---------------------------
// push(bytes) -> array of { id, error, params } for each complete status
// packet found. Resyncs on garbage; never throws.
export class Protocol2Decoder {
  constructor() {
    this._buf = [];
  }

  push(bytes) {
    for (const b of bytes) this._buf.push(b);
    const out = [];

    for (;;) {
      // find header
      let h = -1;
      for (let i = 0; i + 3 < this._buf.length; i++) {
        if (this._buf[i] === 0xff && this._buf[i + 1] === 0xff && this._buf[i + 2] === 0xfd && this._buf[i + 3] === 0x00) {
          h = i;
          break;
        }
      }
      if (h === -1) {
        // keep at most the last 3 bytes (possible partial header)
        if (this._buf.length > 3) this._buf = this._buf.slice(-3);
        break;
      }
      if (h > 0) this._buf = this._buf.slice(h); // drop leading garbage

      if (this._buf.length < 7) break; // need through LEN
      const len = this._buf[5] | (this._buf[6] << 8);
      const total = 7 + len; // header(4)+id(1)+len(2)+len
      if (this._buf.length < total) break;

      const packet = this._buf.slice(0, total);
      this._buf = this._buf.slice(total);

      const gotCrc = packet[total - 2] | (packet[total - 1] << 8);
      const wantCrc = crc16(packet.slice(0, total - 2));
      if (gotCrc !== wantCrc) continue; // corrupt — drop and resync

      const id = packet[4];
      const instruction = packet[7];
      if (instruction !== INSTR.STATUS) continue; // only status packets are inbound
      const error = packet[8] ?? 0;
      const rawParams = packet.slice(9, total - 2);
      out.push({ id, error, params: unstuff(rawParams) });
    }
    return out;
  }
}
