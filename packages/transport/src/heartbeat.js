// startHeartbeat — the same keep-alive loop that was duplicated across
// scripts/prototype-client.mjs and apps/dashboard/index.html, pulled out
// once there were three call sites (the teleop dashboard being the third).
// Includes the send-gap diagnostic added after the unexplained ~23s
// heartbeat stall seen in the first real-browser test (plan.md, "Phase 2
// 진행" section) — any transport using this helper gets that diagnostic
// for free instead of every page reimplementing it.

import { encodeFrame } from './frame.js';
import { CMD } from './commands.js';

function heartbeatFrame(seq) {
  const payload = new Uint8Array(2);
  new DataView(payload.buffer).setUint16(0, seq, true);
  return encodeFrame(CMD.HEARTBEAT, payload);
}

// onGap(gapMs) fires when the time since the previous send exceeds
// gapWarnMs (default 150; the interval itself is 100ms, so this flags a
// tick that was throttled, blocked, or otherwise late).
export function startHeartbeat(transport, { intervalMs = 100, gapWarnMs = 150, onGap, onSend, onSendError } = {}) {
  let seq = 0;
  let lastSendAt = null;

  const timer = setInterval(async () => {
    const now = performance.now();
    const gap = lastSendAt === null ? 0 : now - lastSendAt;
    lastSendAt = now;
    if (gap > gapWarnMs && onGap) onGap(gap);

    try {
      await transport.send(heartbeatFrame(seq++));
      if (onSend) onSend(seq);
    } catch (err) {
      if (onSendError) onSendError(err);
    }
  }, intervalMs);

  return { stop: () => clearInterval(timer) };
}
