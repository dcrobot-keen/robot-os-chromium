// lidar-lds — parser for the TurtleBot3's laser scanner (roadmap.md "세 타깃
// 동시 진행"). The classic Burger ships the ROBOTIS LDS-01 (HLS-LFCD2); this
// decodes its serial packet stream into the SAME scan object shape the
// simulator's sensor stream (:8766) already feeds MapNode, so nothing above
// changes when the scans come from a real laser instead.
//
// LDS-01 wire format: a full 360° revolution is 90 packets of 4 samples.
//   Packet, 22 bytes:
//     0    : 0xFA  start
//     1    : 0xA0..0xF9  -> (index - 0xA0) * 4 = start angle in degrees
//     2..3 : rotation speed, uint16 LE, rpm = value / 64
//     4..21: 4 samples x 4 bytes:
//            b0 = distance bits 0..7 (mm)
//            b1 = distance bits 8..13; bit7 = invalid, bit6 = strength-warning
//            b2..b3 = signal strength, uint16 LE
//     20..21 overlap the last sample's strength? no — 22 = 1+1+2+16+2, the
//            final 2 bytes are the checksum, uint16 LE.
//   Checksum: 10 uint16 LE words over bytes 0..19,
//     chk32 = 0; for each word: chk32 = (chk32 << 1) + word
//     checksum = ((chk32 & 0x7FFF) + (chk32 >> 15)) & 0x7FFF
//
// LDS-02 (newer Burgers) uses a different framing — not handled yet; the
// manifest's `lidar.model` selects, and this throws for anything but lds01.

const DEG = Math.PI / 180;

export const LDS01 = {
  points: 360,
  angleMin: 0,
  angleMax: 2 * Math.PI,
  angleIncrement: (2 * Math.PI) / 360,
  rangeMin: 0.12,
  rangeMax: 3.5,
};

export function ldsChecksum(bytes20) {
  let chk32 = 0;
  for (let i = 0; i < 10; i++) {
    const word = bytes20[2 * i] | (bytes20[2 * i + 1] << 8);
    chk32 = (chk32 << 1) + word;
  }
  return ((chk32 & 0x7fff) + ((chk32 >> 15) & 0x7fff)) & 0x7fff;
}

/**
 * Streaming decoder. push(bytes) -> array of scan objects, one per completed
 * revolution:
 *   { type:'scan', angleMin, angleMax, angleIncrement, rangeMin, rangeMax,
 *     ranges: number[360] (metres; >= rangeMax means no return), rpm }
 */
export class LdsDecoder {
  constructor({ model = 'lds01', spec = LDS01 } = {}) {
    if (model !== 'lds01') {
      throw new Error(`lidar-lds: model "${model}" not supported yet (only lds01)`);
    }
    this._spec = spec;
    this._buf = [];
    this._ranges = new Array(spec.points).fill(spec.rangeMax);
    this._seen = 0; // samples filled since the last revolution boundary
    this._lastIndex = -1;
    this._rpm = 0;
  }

  push(bytes) {
    for (const b of bytes) this._buf.push(b);
    const scans = [];

    while (this._buf.length >= 22) {
      // resync to 0xFA followed by a plausible index byte
      if (this._buf[0] !== 0xfa || this._buf[1] < 0xa0 || this._buf[1] > 0xf9) {
        this._buf.shift();
        continue;
      }
      const pkt = this._buf.slice(0, 22);
      if (ldsChecksum(pkt.slice(0, 20)) !== (pkt[20] | (pkt[21] << 8))) {
        this._buf.shift(); // bad checksum — drop a byte and resync
        continue;
      }
      this._buf = this._buf.slice(22);

      const index = pkt[1] - 0xa0; // 0..89
      this._rpm = (pkt[2] | (pkt[3] << 8)) / 64;

      // a full revolution completed when the index wraps back down
      if (this._lastIndex !== -1 && index <= this._lastIndex) {
        scans.push(this._makeScan());
        this._ranges = new Array(this._spec.points).fill(this._spec.rangeMax);
        this._seen = 0;
      }
      this._lastIndex = index;

      for (let s = 0; s < 4; s++) {
        const o = 4 + s * 4;
        const dLow = pkt[o];
        const dHigh = pkt[o + 1];
        const invalid = (dHigh & 0x80) !== 0;
        const distMm = ((dHigh & 0x3f) << 8) | dLow;
        const angle = index * 4 + s; // degrees, 0..359
        let m = this._spec.rangeMax;
        if (!invalid && distMm > 0) m = distMm / 1000;
        m = Math.max(this._spec.rangeMin, Math.min(this._spec.rangeMax, m));
        this._ranges[angle % this._spec.points] = m;
        this._seen++;
      }
    }
    return scans;
  }

  _makeScan() {
    const { angleMin, angleMax, angleIncrement, rangeMin, rangeMax } = this._spec;
    return {
      type: 'scan',
      angleMin, angleMax, angleIncrement, rangeMin, rangeMax,
      ranges: this._ranges.slice(),
      rpm: this._rpm,
    };
  }
}

/** Build one LDS-01 packet for `index` (0..89) with 4 distances in mm. Test helper. */
export function buildLdsPacket(index, distancesMm, rpm = 300) {
  const speed = Math.round(rpm * 64);
  const bytes = [0xfa, 0xa0 + index, speed & 0xff, (speed >> 8) & 0xff];
  for (let s = 0; s < 4; s++) {
    const d = distancesMm[s] ?? 0;
    bytes.push(d & 0xff, (d >> 8) & 0x3f, 0x20, 0x00); // strength = 32
  }
  const chk = ldsChecksum(bytes.slice(0, 20));
  bytes.push(chk & 0xff, (chk >> 8) & 0xff);
  return new Uint8Array(bytes);
}
