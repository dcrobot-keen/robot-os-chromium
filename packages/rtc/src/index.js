// @ros-chromium/rtc — remote sessions (plan.md Phase 5).
//
//   SignalingClient  — peer-side of apps/signaling-server; rendezvous only.
//   RtcTransport     — operator side; a HardwareTransport over a WebRTC
//                      data channel, so the dashboard drives a remote robot
//                      with the same code it uses locally.
//   RtcHostBridge    — host side; near-transparent byte pipe between an
//                      operator's data channel and the firmware.
//
// SignalingClient runs in the browser and in Node (Node 22+ global
// WebSocket). RtcTransport / RtcHostBridge need RTCPeerConnection, which
// Node does not have — they are browser-only and verified by hand
// (see plan.md). scripts/signaling-smoke.mjs covers the parts Node can run.

export * from './signaling-client.js';
export * from './rtc-transport.js';
export * from './rtc-host-bridge.js';
