// Smoke test for the LDS-01 (HLS-LFCD2) scan parser. No hardware — builds
// packets with the same checksum algorithm the decoder verifies, feeds a
// full revolution, and checks the assembled scan matches the shape MapNode
// already consumes from the simulator's sensor stream.
//
//   node scripts/lidar-lds-smoke.mjs
import { LdsDecoder, buildLdsPacket, ldsChecksum, LDS01 } from '../packages/transport/src/lidar-lds.js';

let failures = 0;
const check = (name, cond, extra = '') => {
  console.log(`${cond ? 'PASS' : 'FAIL'}  ${name}${extra ? '  (' + extra + ')' : ''}`);
  if (!cond) failures++;
};

// --- 1. checksum round-trips (build uses the same algo the decoder checks) ---
{
  const pkt = buildLdsPacket(0, [1000, 1100, 1200, 1300]);
  check('buildLdsPacket: 22 bytes', pkt.length === 22);
  check('buildLdsPacket: checksum in the last two bytes matches ldsChecksum',
    ldsChecksum([...pkt.slice(0, 20)]) === (pkt[20] | (pkt[21] << 8)));
}

// --- 2. a full revolution -> one scan of 360 metres-ranges -------------
{
  const dec = new LdsDecoder();
  let scans = [];
  // 90 packets, each 4 samples; sample at angle a gets distance (500 + a) mm.
  // Start a *second* revolution so the wrap boundary fires and flushes the first.
  for (let rev = 0; rev < 2; rev++) {
    for (let index = 0; index < 90; index++) {
      const d = [0, 1, 2, 3].map((s) => 500 + index * 4 + s);
      scans = scans.concat(dec.push(buildLdsPacket(index, d)));
    }
  }
  check('LdsDecoder: exactly one scan flushed after the first full revolution', scans.length === 1, `${scans.length}`);
  const scan = scans[0];
  check('LdsDecoder: scan has 360 ranges', scan.ranges.length === 360);
  check('LdsDecoder: scan shape matches the sim sensor stream',
    scan.angleMin === LDS01.angleMin && scan.angleMax === LDS01.angleMax
      && Math.abs(scan.angleIncrement - LDS01.angleIncrement) < 1e-12
      && scan.rangeMax === LDS01.rangeMax);
  check('LdsDecoder: mm -> m, correct beam', Math.abs(scan.ranges[10] - 0.510) < 1e-9, `ranges[10]=${scan.ranges[10]}`);
  check('LdsDecoder: distances clamp into [rangeMin, rangeMax]',
    scan.ranges.every((r) => r >= LDS01.rangeMin - 1e-9 && r <= LDS01.rangeMax + 1e-9));
}

// --- 3. invalid-flag and zero distance become "no return" (rangeMax) ---
{
  const dec = new LdsDecoder();
  // packet at index 5: sample 0 has the invalid bit set, sample 1 is zero
  const speed = 300 * 64;
  const bytes = [0xfa, 0xa0 + 5, speed & 0xff, (speed >> 8) & 0xff];
  bytes.push(0xff, 0x80 | 0x3f, 0x20, 0x00); // invalid flag set
  bytes.push(0x00, 0x00, 0x20, 0x00); // zero distance
  bytes.push(0xe8, 0x03, 0x20, 0x00); // 1000 mm
  bytes.push(0xe8, 0x03, 0x20, 0x00); // 1000 mm
  const chk = ldsChecksum(bytes.slice(0, 20));
  bytes.push(chk & 0xff, (chk >> 8) & 0xff);

  dec.push(new Uint8Array(bytes));
  // force a revolution flush with a wrap
  const scans = dec.push(buildLdsPacket(0, [1000, 1000, 1000, 1000]));
  const scan = scans[0];
  check('LdsDecoder: invalid-flagged sample -> rangeMax', scan.ranges[20] === LDS01.rangeMax, `ranges[20]=${scan.ranges[20]}`);
  check('LdsDecoder: zero-distance sample -> rangeMax', scan.ranges[21] === LDS01.rangeMax, `ranges[21]=${scan.ranges[21]}`);
  check('LdsDecoder: valid sample in the same packet still parses', Math.abs(scan.ranges[22] - 1.0) < 1e-9, `ranges[22]=${scan.ranges[22]}`);
}

// --- 4. resyncs past a corrupt byte ------------------------------------
{
  const dec = new LdsDecoder();
  const good = buildLdsPacket(0, [800, 800, 800, 800]);
  const junk = new Uint8Array([0x00, 0x13, 0xfa, 0x37]);
  dec.push(junk);
  dec.push(good);
  const scans = dec.push(buildLdsPacket(0, [800, 800, 800, 800])); // wrap -> flush
  check('LdsDecoder: recovers a scan after leading junk bytes', scans.length === 1 && scans[0].ranges[0] > 0);
}

console.log(failures === 0 ? '\nall lidar-lds smoke checks passed' : `\n${failures} check(s) failed`);
process.exit(failures === 0 ? 0 : 1);
