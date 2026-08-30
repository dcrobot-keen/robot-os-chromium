// codecs.js — wire-protocol codec registry, keyed by a manifest's
// `transport.kind`. This is the one piece of plumbing that lets the same
// stack drive three targets (simulator, TurtleBot3, Former) — see
// roadmap.md "세 타깃 동시 진행".
//
// A codec is { Decoder, encode }:
//   Decoder     — class; `new Decoder().push(bytes)` yields normalized
//                 messages ({type:'ack'|'reply'|'line', ...}). The shape is
//                 the SAME across codecs, so packages/nodes and
//                 device-abstraction never see the wire protocol.
//   encode(spec) -> Uint8Array — turn one command into wire bytes.
//
// Today the only wire protocol is the Former/simulator's Roboteq ASCII line
// protocol (former-motor-protocol.md). A second target — TurtleBot3's
// OpenCR, DYNAMIXEL Protocol 2.0 — registers here as another entry
// ('turtlebot3-opencr') with a Decoder that emits the same {type:'reply',
// key, values} messages (e.g. key:'C' for encoder counts, so OdometryNode
// is unchanged), and nothing above HardwareTransport moves.

import { RoboteqDecoder, encodeCommand } from './roboteq.js';
import { makeTurtlebot3OpenCRCodec, opencrConfigFromManifest } from './turtlebot3-opencr.js';

// A registry entry is either a plain { Decoder, encode } (stateless — reused
// across transports, like Roboteq) or { factory(manifest) -> { Decoder,
// encode } } for codecs that need per-transport state (TB3's OpenCR, whose
// encode/Decoder share a request queue since Protocol 2.0 status packets
// don't echo the address they answer).
export const CODECS = {
  'roboteq-serial': { Decoder: RoboteqDecoder, encode: encodeCommand },
  'turtlebot3-opencr': {
    factory: (manifest) => makeTurtlebot3OpenCRCodec(opencrConfigFromManifest(manifest ?? {})),
  },
};

export const DEFAULT_CODEC_KIND = 'roboteq-serial';

/**
 * Look up a codec by `transport.kind`. Defaults to Roboteq so every existing
 * call site (which passes nothing) is unchanged. Pass the manifest for
 * factory codecs that read addresses/ids out of it (turtlebot3-opencr).
 * @param {string} [kind]
 * @param {object} [manifest]
 * @returns {{ Decoder: new () => { push(bytes: Uint8Array): Iterable<object> }, encode: (spec: any) => Uint8Array }}
 */
export function getCodec(kind = DEFAULT_CODEC_KIND, manifest) {
  const entry = CODECS[kind];
  if (!entry) {
    throw new Error(
      `unknown transport codec kind "${kind}" — registered: ${Object.keys(CODECS).join(', ')}`,
    );
  }
  return entry.factory ? entry.factory(manifest) : entry;
}
