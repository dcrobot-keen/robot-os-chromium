// Command vocabulary — the entire surface the browser/host is allowed to
// touch (research.md, "Wire protocol" table). Only the three needed for the
// simplest prototype are assigned; the rest are reserved for when
// GET_ENCODER / GET_IMU / GET_BATTERY telemetry gets added.
//
// NOTE: this file is intentionally duplicated in
// firmware/sim/src/commands.js — the two repos don't share a
// filesystem-relative import across the repo boundary on purpose (each repo
// must stand alone). Keep both copies in sync until a shared protocol
// spec/codegen replaces the duplication.

export const CMD = Object.freeze({
  HEARTBEAT: 0x01,
  SET_VELOCITY: 0x02,
  ESTOP: 0x03,
  // reserved, not yet implemented:
  GET_ENCODER: 0x04,
  GET_IMU: 0x05,
  GET_BATTERY: 0x06,
});
