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
  constructor(bus, { topic, maxSpeed = 1.0, pollMs = 50, gamepadIndex = 0 } = {}) {
    if (!topic) throw new Error('TeleopNode requires a topic');
    this._bus = bus;
    this._topic = topic;
    this._maxSpeed = maxSpeed;
    this._pollMs = pollMs;
    this._gamepadIndex = gamepadIndex;
    this._timer = null;
  }

  start() {
    this._timer = setInterval(() => this._tick(), this._pollMs);
  }

  stop() {
    clearInterval(this._timer);
  }

  connectedGamepad() {
    return navigator.getGamepads()[this._gamepadIndex] ?? null;
  }

  _tick() {
    const pad = this.connectedGamepad();
    if (!pad) return;

    const forward = applyDeadzone(-pad.axes[1]); // stick up reports negative Y
    const turn = applyDeadzone(pad.axes[0]);

    const left = clamp(forward + turn, -1, 1) * this._maxSpeed;
    const right = clamp(forward - turn, -1, 1) * this._maxSpeed;

    this._bus.publish(this._topic, { left, right });
  }
}
