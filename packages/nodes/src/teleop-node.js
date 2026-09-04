// TeleopNode — polls the Gamepad API and publishes differential-drive
// velocity commands onto the bus. Polling, not events, because the
// Gamepad API has no "axis changed" event; the only way to read stick
// position is navigator.getGamepads() on a timer (architecture doc, Layer
// 04).
//
// Mixing is arcade-style: left stick Y is forward/back, X is turn — not
// literal per-stick-per-wheel control.

const DEADZONE = 0.08;

function clamp(v, lo, hi) {
  return Math.min(hi, Math.max(lo, v));
}

function applyDeadzone(v) {
  return Math.abs(v) < DEADZONE ? 0 : v;
}

export class TeleopNode {
  // onTick(pad), if given, fires on every poll (pad is null when no
  // gamepad is present) — added to diagnose a real-browser test where
  // publishes arrived far less often, and with far more value repetition,
  // than a 50ms unconditional-publish loop should ever produce (see
  // plan.md, "실사용 테스트 — 게임패드 조작 확인"). A Node test with a
  // mocked navigator.getGamepads() showed the tick loop itself firing
  // close to the configured interval, so the discrepancy is suspected to
  // be Chrome's Gamepad API state only refreshing on animation frames
  // rather than on an arbitrary setInterval — onTick lets that be checked
  // directly instead of guessed at.
  constructor(bus, { topic, maxSpeed = 1.0, pollMs = 50, gamepadIndex = 0, onTick } = {}) {
    if (!topic) throw new Error('TeleopNode requires a topic');
    this._bus = bus;
    this._topic = topic;
    this._maxSpeed = maxSpeed;
    this._pollMs = pollMs;
    this._gamepadIndex = gamepadIndex;
    this._onTick = onTick;
    this._timer = null;
  }

  start() {
    this._timer = setInterval(() => this._tick(), this._pollMs);
  }

  stop() {
    clearInterval(this._timer);
  }

  // Preferred slot first, then any connected pad. Chrome hands out slots in
  // connection order and keeps them after disconnects, so a pad that shows
  // up as "(index 1)" is common; polling only [0] then silently sends zeros.
  connectedGamepad() {
    const pads = navigator.getGamepads();
    return pads[this._gamepadIndex] ?? Array.from(pads).find(Boolean) ?? null;
  }

  _tick() {
    const pad = this.connectedGamepad();
    if (this._onTick) this._onTick(pad);
    if (!pad) return;

    const forward = applyDeadzone(-pad.axes[1]); // stick up reports negative Y
    const turn = applyDeadzone(pad.axes[0]);

    const left = clamp(forward + turn, -1, 1) * this._maxSpeed;
    const right = clamp(forward - turn, -1, 1) * this._maxSpeed;

    this._bus.publish(this._topic, { left, right });
  }
}
