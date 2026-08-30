// turtlebot3-opencr — the codec (a { Decoder, encode } pair) that lets the
// stack drive a TurtleBot3 Burger's OpenCR board, stock firmware
// (roadmap.md "세 타깃 동시 진행", Option A). OpenCR presents as one
// DYNAMIXEL device (default id 200) with a control table; this maps our
// HardwareTransport calls onto Protocol 2.0 reads/writes and turns the
// replies back into the SAME normalized messages roboteq.js emits, so
// packages/nodes (OdometryNode especially) is unchanged.
//
// !!! The control-table addresses live in the manifest, not here, and MUST
// be verified against the specific OpenCR firmware on the robot before the
// first hardware run — see todo-tb3.md. This file only knows the shapes.
//
// Because Protocol 2.0 status packets don't echo which address a READ was
// for, encode() and the Decoder share a small FIFO of outstanding requests
// (the same request/response ordering roboteq-smoke already relies on).
// That shared state is why codecs.js builds this codec per-transport via a
// factory rather than reusing one object.

import {
  INSTR, Protocol2Decoder, buildRead, buildWrite, buildPing, toLE, fromLE,
} from './dynamixel-protocol2.js';

/**
 * @param {object} config
 * @param {number} [config.id=200] - the OpenCR's DYNAMIXEL id
 * @param {{[key:string]: { address:number, fields:Array<{name:string,offset:number,width:number,signed?:boolean}> }}} config.reads
 *   - named read groups. OdometryNode's encoderQuery is `{op:'read', key:'C'}`,
 *     so `reads.C` must describe the two encoder words.
 * @param {{[name:string]: { address:number, width:number, signed?:boolean }}} [config.controlTable]
 *   - named single control-table entries, for `{op:'write', from:'<name>', value}`.
 * @returns {{ Decoder: new () => object, encode: (spec:any) => Uint8Array }}
 */
export function makeTurtlebot3OpenCRCodec({ id = 200, reads = {}, controlTable = {} } = {}) {
  const pending = []; // { kind:'read'|'write'|'ping', key?, fields? }

  function readLength(fields) {
    return Math.max(...fields.map((f) => f.offset + f.width));
  }

  function encode(spec) {
    if (typeof spec === 'string') {
      throw new Error(`turtlebot3-opencr.encode expects a structured op, got string "${spec}" — the manifest's drive.commands must be objects for this codec`);
    }
    switch (spec.op) {
      case 'read': {
        const r = reads[spec.key];
        if (!r) throw new Error(`turtlebot3-opencr: no read group "${spec.key}" configured`);
        pending.push({ kind: 'read', key: spec.key, fields: r.fields });
        return buildRead(id, r.address, readLength(r.fields));
      }
      case 'write': {
        let address = spec.address;
        let fields = spec.fields;
        if (spec.from) {
          const ct = controlTable[spec.from];
          if (!ct) throw new Error(`turtlebot3-opencr: no control-table entry "${spec.from}"`);
          address = ct.address;
          if (fields === undefined) {
            fields = [{ value: spec.value, width: ct.width, signed: ct.signed }];
          }
        }
        if (address === undefined || !fields) {
          throw new Error('turtlebot3-opencr write needs {address, fields} or {from, value}');
        }
        const data = [];
        for (const f of fields) data.push(...toLE(Number(f.value), f.width));
        pending.push({ kind: 'write' });
        return buildWrite(id, address, data);
      }
      case 'ping':
        pending.push({ kind: 'ping' });
        return buildPing(id);
      default:
        throw new Error(`turtlebot3-opencr: unknown op "${spec.op}"`);
    }
  }

  class Decoder {
    constructor() {
      this._p2 = new Protocol2Decoder();
    }

    push(bytes) {
      const out = [];
      for (const status of this._p2.push(bytes)) {
        const req = pending.shift();
        if (!req || req.kind !== 'read') {
          // write / ping ack, or an unsolicited status
          out.push({ type: 'ack', ok: status.error === 0, raw: status.params });
          continue;
        }
        const values = req.fields.map((f) =>
          fromLE(status.params.slice(f.offset, f.offset + f.width), { signed: !!f.signed }),
        );
        out.push({ type: 'reply', key: req.key, values, raw: status.params });
      }
      return out;
    }
  }

  return { Decoder, encode };
}

/**
 * Build the codec config from a manifest. `transport.controlTable` names the
 * addresses; the encoder read group "C" is assembled from the two
 * present-position entries (they must be contiguous for a single READ — the
 * common OpenCR layout; if a firmware splits them, extend this).
 */
export function opencrConfigFromManifest(manifest) {
  const t = manifest.transport ?? {};
  const ct = t.controlTable ?? {};
  const L = ct.presentPositionLeft;
  const R = ct.presentPositionRight;
  const reads = {};
  if (L && R) {
    const lo = Math.min(L.address, R.address);
    reads.C = {
      address: lo,
      fields: [
        { name: 'left', offset: L.address - lo, width: L.width, signed: !!L.signed },
        { name: 'right', offset: R.address - lo, width: R.width, signed: !!R.signed },
      ],
    };
  }
  return { id: t.dxlId ?? 200, reads, controlTable: ct };
}
