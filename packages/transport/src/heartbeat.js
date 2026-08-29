// startHeartbeat — the keepalive loop. On the Former's Roboteq base the
// liveness signal is simply "keep sending commands": the controller's
// serial watchdog (RWD, ~1s) stops the motors if the wire goes quiet.
// This sends "!B 3 1" every 100ms — the same keepalive bool the reference
// ROS driver writes every control cycle, also watched by the onboard
// safety script (former-motor-protocol.md).
//
// Keeps the send-gap diagnostic added after the unexplained ~23s heartbeat
// stall in the first real-browser test (plan.md, "Phase 2 진행"): onGap
// fires when the gap since the previous send exceeds gapWarnMs.

import { encodeCommand, cmd } from './roboteq.js';

export function startHeartbeat(transport, { intervalMs = 100, gapWarnMs = 150, onGap, onSend, onSendError } = {}) {
  let lastSendAt = null;

  const timer = setInterval(async () => {
    const now = performance.now();
    const gap = lastSendAt === null ? 0 : now - lastSendAt;
    lastSendAt = now;
    if (gap > gapWarnMs && onGap) onGap(gap);

    try {
      await transport.send(encodeCommand(cmd.keepAlive()));
      if (onSend) onSend();
    } catch (err) {
      if (onSendError) onSendError(err);
    }
  }, intervalMs);

  return { stop: () => clearInterval(timer) };
}
