// Smoke test for the DYNAMIXEL Protocol 2.0 wire codec + the turtlebot3-opencr
// codec layer on top of it. No hardware — checks framing/CRC against the
// ROBOTIS reference vectors, byte stuffing, and that a status packet
// carrying encoder bytes comes back out of the codec as the SAME normalized
// {type:'reply', key:'C', values:[left,right]} shape roboteq.js produces, so
// OdometryNode is unchanged across the two robots.
//
//   node scripts/dynamixel-smoke.mjs
import { readFile } from 'node:fs/promises';
import {
  crc16, toLE, fromLE, stuff, unstuff,
  buildPing, buildWrite, buildRead, buildInstruction, INSTR, Protocol2Decoder,
} from '../packages/transport/src/dynamixel-protocol2.js';
import { makeTurtlebot3OpenCRCodec, opencrConfigFromManifest } from '../packages/transport/src/turtlebot3-opencr.js';
import { getCodec } from '../packages/transport/src/codecs.js';
import { createDriveDevice } from '../packages/device-abstraction/src/index.js';

let failures = 0;
const check = (name, cond, extra = '') => {
  console.log(`${cond ? 'PASS' : 'FAIL'}  ${name}${extra ? '  (' + extra + ')' : ''}`);
  if (!cond) failures++;
};
const hex = (u8) => [...u8].map((b) => b.toString(16).padStart(2, '0')).join(' ');

// --- 1. reference vector: PING id 1 -------------------------------------
{
  const got = buildPing(1);
  const want = 'ff ff fd 00 01 03 00 01 19 4e';
  check('PING id 1 matches the ROBOTIS reference packet', hex(got) === want, hex(got));
}

// --- 2. reference vector: WRITE id 1 addr 116 value 512 ---------------
{
  const got = buildWrite(1, 116, toLE(512, 4));
  const want = 'ff ff fd 00 01 09 00 03 74 00 00 02 00 00 ca 89';
  check('WRITE id 1 addr 116 = 512 matches the reference packet', hex(got) === want, hex(got));
}

// --- 3. LE round-trips, signed ---------------------------------------
{
  check('toLE/fromLE: 4-byte unsigned', fromLE(toLE(305419896, 4)) === 305419896);
  check('toLE/fromLE: 4-byte signed negative', fromLE(toLE(-12345, 4), { signed: true }) === -12345);
  check('toLE: little-endian order', hex(new Uint8Array(toLE(0x12345678, 4))) === '78 56 34 12');
}

// --- 4. byte stuffing round-trip -----------------------------------
{
  const raw = [0x01, 0xff, 0xff, 0xfd, 0x03, 0xff, 0xff, 0xfd, 0xfd, 0x04];
  const s = stuff(raw);
  check('stuff inserts 0xfd after FF FF FD', s.length === raw.length + 2, `+${s.length - raw.length}`);
  check('unstuff(stuff(x)) === x', JSON.stringify(unstuff(s)) === JSON.stringify(raw), hex(new Uint8Array(unstuff(s))));
}

// --- 5. Protocol2Decoder parses a STATUS packet, tolerates leading garbage ---
{
  // STATUS from id 200, no error, params = 8 bytes (two int32 LE: 1000, -250)
  const params = [...toLE(1000, 4), ...toLE(-250, 4)];
  const status = buildInstruction({ id: 200, instruction: INSTR.STATUS, params: [0x00, ...params] });
  const dec = new Protocol2Decoder();
  const out = dec.push(new Uint8Array([0xaa, 0xbb, ...status])); // garbage prefix
  check('Protocol2Decoder: one status packet recovered past garbage', out.length === 1 && out[0].id === 200);
  check('Protocol2Decoder: status params carry the 8 payload bytes', out[0].params.length === 8);
  check('Protocol2Decoder: split across two push()es still parses',
    (() => {
      const d = new Protocol2Decoder();
      const a = d.push(status.slice(0, 5));
      const b = d.push(status.slice(5));
      return a.length === 0 && b.length === 1 && b[0].id === 200;
    })());
  check('Protocol2Decoder: a corrupt CRC is dropped, not emitted',
    (() => {
      const bad = Uint8Array.from(status);
      bad[bad.length - 1] ^= 0xff;
      return new Protocol2Decoder().push(bad).length === 0;
    })());
}

// --- 6. turtlebot3-opencr codec: encoder read round-trips to {key:'C'} ---
{
  const codec = makeTurtlebot3OpenCRCodec({
    id: 200,
    // minimal control-table map — real addresses are verified on hardware
    // (todo-tb3.md); the smoke only needs the shape to be right.
    reads: {
      C: { address: 132, fields: [
        { name: 'left', offset: 0, width: 4, signed: true },
        { name: 'right', offset: 4, width: 4, signed: true },
      ] },
    },
  });
  const decoder = new codec.Decoder();

  // OdometryNode does: transport.send(transport.encode(encoderQuery))
  const readPkt = codec.encode({ op: 'read', key: 'C' });
  check('opencr encode({op:read,key:C}) -> a READ instruction packet',
    readPkt[7] === INSTR.READ && readPkt[0] === 0xff);

  // OpenCR replies with a STATUS packet carrying 8 bytes (left=1500, right=1490)
  const reply = buildInstruction({
    id: 200, instruction: INSTR.STATUS,
    params: [0x00, ...toLE(1500, 4), ...toLE(1490, 4)],
  });
  const msgs = decoder.push(reply);
  check('opencr Decoder emits the roboteq-shaped {type:reply, key:C, values:[l,r]}',
    msgs.length === 1 && msgs[0].type === 'reply' && msgs[0].key === 'C'
      && msgs[0].values[0] === 1500 && msgs[0].values[1] === 1490,
    JSON.stringify(msgs[0]));

  // a WRITE (setVelocity / torque-enable) round-trips to a WRITE packet + '+' ack
  const writePkt = codec.encode({ op: 'write', address: 15, fields: [
    { value: 200, width: 4, signed: true },
    { value: 180, width: 4, signed: true },
  ] });
  check('opencr encode({op:write}) -> a WRITE instruction packet', writePkt[7] === INSTR.WRITE);
  const wAck = decoder.push(buildInstruction({ id: 200, instruction: INSTR.STATUS, params: [0x00] }));
  check('opencr Decoder maps an empty status to {type:ack, ok:true}',
    wAck.length === 1 && wAck[0].type === 'ack' && wAck[0].ok === true);
  const eAck = decoder.push(buildInstruction({ id: 200, instruction: INSTR.STATUS, params: [0x01] }));
  check('opencr Decoder maps a status with an error byte to {type:ack, ok:false}',
    eAck.length === 1 && eAck[0].type === 'ack' && eAck[0].ok === false);
}

// --- 7. codecs.js factory: getCodec('turtlebot3-opencr', manifest) -----
{
  const manifest = JSON.parse(await readFile(new URL('../manifests/tb3.manifest.json', import.meta.url), 'utf-8'));
  const codec = getCodec(manifest.transport.kind, manifest);
  check('getCodec builds the opencr codec from the manifest kind', typeof codec.encode === 'function' && typeof codec.Decoder === 'function');
  const cfg = opencrConfigFromManifest(manifest);
  check('opencrConfigFromManifest: reads.C spans both present-position words',
    cfg.reads.C && cfg.reads.C.fields.length === 2
      && cfg.reads.C.fields[0].offset === 0 && cfg.reads.C.fields[1].offset === 4,
    JSON.stringify(cfg.reads.C));
}

// --- 8. createDriveDevice over the tb3 manifest -> Protocol 2.0 packets ---
{
  const manifest = JSON.parse(await readFile(new URL('../manifests/tb3.manifest.json', import.meta.url), 'utf-8'));
  const codec = getCodec(manifest.transport.kind, manifest);
  const sent = [];
  const fakeTransport = {
    encode: (spec) => codec.encode(spec),
    send: (bytes) => { sent.push(bytes); },
    onMessage() {},
  };
  const drive = createDriveDevice(fakeTransport, manifest);

  await drive.enable();
  check('tb3 drive.enable() -> a WRITE to torqueEnable (addr 149, value 1)',
    sent.length === 1 && sent[0][7] === INSTR.WRITE
      && (sent[0][8] | (sent[0][9] << 8)) === 149 && sent[0][10] === 1,
    hex(sent[0]));

  sent.length = 0;
  await drive.setVelocity(0.5, -0.25); // normalized -> scale 1000 -> 500, -250
  const pkt = sent[0];
  const addr = pkt[8] | (pkt[9] << 8);
  const data = [...pkt.slice(10, 18)];
  check('tb3 drive.setVelocity(0.5,-0.25) -> WRITE goalVelocityLeft, two int32 = 500, -250',
    pkt[7] === INSTR.WRITE && addr === 140
      && fromLE(data.slice(0, 4), { signed: true }) === 500
      && fromLE(data.slice(4, 8), { signed: true }) === -250,
    `addr=${addr} data=${hex(new Uint8Array(data))}`);
}

// --- 9. Roboteq path is unchanged (string commands still -> ASCII + CR) ---
{
  const manifest = JSON.parse(await readFile(new URL('../manifests/former.manifest.json', import.meta.url), 'utf-8'));
  const codec = getCodec(manifest.transport.kind); // 'roboteq-serial'
  const sent = [];
  const drive = createDriveDevice(
    { encode: (s) => codec.encode(s), send: (b) => sent.push(b), onMessage() {} },
    manifest,
  );
  await drive.setVelocity(0.5, -0.5);
  const line = new TextDecoder().decode(sent[0]);
  check('former (roboteq) drive.setVelocity still emits "!G 1 500_!G 2 -500\\r"',
    line === '!G 1 500_!G 2 -500\r', JSON.stringify(line));
}

console.log(failures === 0 ? '\nall dynamixel smoke checks passed' : `\n${failures} check(s) failed`);
process.exit(failures === 0 ? 0 : 1);
