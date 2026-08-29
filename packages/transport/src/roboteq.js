// Roboteq motor-controller serial codec — the real wire protocol of the
// Former 2.0 base (see ../../../../former-motor-protocol.md). ASCII, one
// command per line terminated by CR ("\r"); multiple sub-commands joined
// with "_"; replies are "+"/"-" acks or "KEY=v1:v2" / "KEY:v1:v2" lines.
//
// Replaces the made-up SOF/LEN/CMD/CRC16 binary frame (frame.js +
// commands.js), which was always a placeholder until the target robot was
// known. Deliberately duplicated in firmware/sim/src/roboteq.js — the two
// repos don't share a filesystem path on purpose; keep the copies
// byte-for-byte identical.

const CR = '\r';
const _enc = new TextEncoder();

// Bytes for one command line. `line` has no trailing CR, e.g.
// "!G 1 500_!G 2 -500" or "?C".
export function encodeCommand(line) {
  return _enc.encode(line + CR);
}

// Command/query string builders — thin templates so call sites read the
// way former-motor-protocol.md does.
export const cmd = {
  echoOff:       ()       => '^ECHOF 1',
  queryFirmware: ()       => '?FID',
  restartScript: (n = 2)  => `!R ${n}`,
  keepAlive:     ()       => '!B 3 1',
  setAccel:      (n)      => `!AC 1 ${n}_!AC 2 ${n}`,
  setDecel:      (n)      => `!DC 1 ${n}_!DC 2 ${n}`,
  resetEncoders: ()       => '!C 1 0_!C 2 0',
  motorGo:       ()       => '!MG',
  estop:         ()       => '!EX',
  motorCommand:  (l, r, chL = 1, chR = 2) => `!G ${chL} ${l}_!G ${chR} ${r}`,
  queryRuntime:  ()       => '?A_?AI_?C_?FF_?T 1_?V 2_?DI',
};

// Streaming line decoder: feed raw bytes as they arrive, get parsed
// messages out. A message is one of:
//   { type: 'ack',   ok: true|false }               from "+" / "-"
//   { type: 'reply', key, values: number[], raw }   from "KEY=1:2" / "KEY:1:2"
//   { type: 'line',  raw }                           anything else non-empty
//     (this is also how an inbound *command* line looks to the sim, which
//      then splits raw on "_")
export class RoboteqDecoder {
  constructor() {
    this._buf = '';
    this._dec = new TextDecoder();
  }

  push(bytes) {
    this._buf += this._dec.decode(bytes, { stream: true });
    const out = [];
    let nl;
    while ((nl = this._buf.indexOf(CR)) !== -1) {
      const line = this._buf.slice(0, nl).trim();
      this._buf = this._buf.slice(nl + 1);
      if (line === '') continue;
      if (line === '+') { out.push({ type: 'ack', ok: true }); continue; }
      if (line === '-') { out.push({ type: 'ack', ok: false }); continue; }
      const sep = line.search(/[=:]/);
      if (sep > 0) {
        const key = line.slice(0, sep);
        const rest = line.slice(sep + 1);
        const values = rest === '' ? [] : rest.split(':').map((v) => Number(v));
        out.push({ type: 'reply', key, values, raw: line });
      } else {
        out.push({ type: 'line', raw: line });
      }
    }
    return out;
  }
}
