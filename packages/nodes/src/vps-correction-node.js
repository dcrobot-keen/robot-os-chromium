// VpsCorrectionNode -- the real-robot replacement for the "ground truth +
// noise" correction stand-in that sim-driver / nav.html feed PoseFusionNode.
// Every `periodMs` it grabs a camera frame (`capture`), asks vps-system's
// POST /localize (`localize`), turns the ARKit-frame answer into a 2D pose on
// the map plane (`toPose`) and publishes it on the correction topic. That is
// exactly the input PoseFusionNode already fuses with odometry, so nothing
// downstream changes between the simulator loop and the real robot.
//
// Camera and HTTP are INJECTED (capture(): Promise<Blob>, localize(blob):
// Promise<result>) so the node runs in the browser (getUserMedia + fetch),
// in Node (any image source) and in the smoke test (fakes) unchanged.
//
// Frames:  /localize returns translation/quaternion in the matched room's
// ARKit world (Y-up). arkitPoseToGroundPose() drops to the slice plane
// (x, y) = (x, -z) with heading from the camera's forward vector (the same
// maths as pathfinder/src/livePoseTransform.js and the app's
// GroundPose.fromARKitTransform). If the server also returns `frame`
// (the room's ScanAlignment inside its group, see vps-system server
// app/frames.py), applyFrame() moves the pose into the group reference
// frame -- the plane pathfinder projects made from the merged slicemap use.

/** ARKit (Y-up) camera pose -> slice-plane pose {x, y, theta}. */
export function arkitPoseToGroundPose(translation, quaternion) {
  const [qx, qy, qz, qw] = quaternion;
  const x = translation[0];
  const y = -translation[2];
  // world direction of the camera's local -Z (forward) = -(third column of R(q))
  const forwardX = -2 * (qx * qz + qy * qw);
  const forwardZ = 2 * (qx * qx + qy * qy) - 1;
  return { x, y, theta: Math.atan2(-forwardZ, forwardX) };
}

/**
 * Move a slice-plane pose from a room's own frame into the group reference
 * frame using the room's ScanAlignment {offsetX, offsetZ, yawRadians}
 * (scan-group-alignment-v1: ARKit plane x' = x cos + z sin + offsetX,
 * z' = -x sin + z cos + offsetZ; slice plane y = -z). Heading is carried by
 * transforming a point one metre ahead, so no sign convention can drift.
 */
export function applyFrame(pose, frame) {
  if (!frame) return pose;
  const { offsetX = 0, offsetZ = 0, yawRadians = 0 } = frame;
  const c = Math.cos(yawRadians), s = Math.sin(yawRadians);
  const xz = (x, y) => {
    const z = -y;
    return [x * c + z * s + offsetX, -x * s + z * c + offsetZ];
  };
  const [x1, z1] = xz(pose.x, pose.y);
  const [x2, z2] = xz(pose.x + Math.cos(pose.theta), pose.y + Math.sin(pose.theta));
  return { x: x1, y: -z1, theta: Math.atan2(-(z2 - z1), x2 - x1) };
}

/** /localize JSON -> correction pose, or null when the answer is unusable. */
export function vpsResultToPose(result, { minInliers = 12 } = {}) {
  if (!result || !Array.isArray(result.translation) || !Array.isArray(result.quaternion)) return null;
  if (typeof result.num_inliers === 'number' && result.num_inliers < minInliers) return null;
  const ground = arkitPoseToGroundPose(result.translation, result.quaternion);
  const pose = applyFrame(ground, result.frame?.alignment ?? null);
  return { ...pose, roomId: result.room_id ?? null, mapId: result.frame?.mapId ?? result.frame?.group ?? null, inliers: result.num_inliers ?? null };
}

/**
 * fetch()-based client for vps-system POST /localize (multipart: image +
 * fx, fy, cx, cy, width, height). Returns the parsed JSON or throws with the
 * server's detail (422 = "no fix", a normal outcome the node just skips).
 */
export function createVpsLocalizeClient(baseUrl, intrinsics, { fetchImpl = globalThis.fetch } = {}) {
  const { fx, fy, cx, cy, width, height } = intrinsics;
  for (const [k, v] of Object.entries({ fx, fy, cx, cy, width, height })) {
    if (typeof v !== 'number' || !Number.isFinite(v)) throw new Error(`intrinsics.${k} must be a number`);
  }
  const url = `${String(baseUrl).replace(/\/$/, '')}/localize`;
  return async function localize(imageBlob) {
    const form = new FormData();
    form.append('image', imageBlob, 'frame.jpg');
    for (const [k, v] of Object.entries({ fx, fy, cx, cy, width, height })) form.append(k, String(v));
    const res = await fetchImpl(url, { method: 'POST', body: form });
    const body = await res.json().catch(() => ({}));
    if (!res.ok) {
      const err = new Error(body.detail || `localize failed (${res.status})`);
      err.status = res.status;
      throw err;
    }
    return body;
  };
}

export class VpsCorrectionNode {
  /**
   * @param {{publish: Function}} bus
   * @param {{
   *   capture: () => Promise<Blob|ArrayBuffer|Uint8Array>,
   *   localize: (image: any) => Promise<object>,
   *   correctionTopic: string,
   *   periodMs?: number,        // default 2000 (a real /localize takes 0.5-1 s)
   *   minInliers?: number,
   *   toPose?: (result) => pose|null,
   *   onFix?: (pose, result) => void,
   *   onMiss?: (err) => void,   // 422 / network errors; the node keeps trying
   *   log?: (msg) => void,
   *   setTimer?: (fn, ms) => any, clearTimer?: (h) => void   // injectable for tests
   * }} options
   */
  constructor(bus, options = {}) {
    const {
      capture, localize, correctionTopic, periodMs = 2000, minInliers = 12, toPose, onFix = () => {}, onMiss = () => {}, log = () => {},
      setTimer = setTimeout, clearTimer = clearTimeout,
    } = options;
    if (typeof capture !== 'function' || typeof localize !== 'function' || !correctionTopic) {
      throw new Error('VpsCorrectionNode requires capture(), localize() and correctionTopic');
    }
    this._bus = bus;
    this._capture = capture;
    this._localize = localize;
    this._topic = correctionTopic;
    this._periodMs = periodMs;
    this._toPose = toPose ?? ((r) => vpsResultToPose(r, { minInliers }));
    this._onFix = onFix;
    this._onMiss = onMiss;
    this._log = log;
    this._setTimer = setTimer;
    this._clearTimer = clearTimer;
    this._timer = null;
    this._running = false;
    this._busy = false;
    this.stats = { attempts: 0, fixes: 0, misses: 0, lastFixAt: null, lastError: null };
  }

  start() {
    if (this._running) return;
    this._running = true;
    this._log(`VPS correction every ${this._periodMs} ms -> ${this._topic}`);
    this._schedule(0);
  }

  stop() {
    this._running = false;
    if (this._timer !== null) this._clearTimer(this._timer);
    this._timer = null;
  }

  _schedule(ms) {
    if (!this._running) return;
    this._timer = this._setTimer(() => this.tick(), ms);
  }

  /** One capture -> localize -> publish cycle. Public so tests / a "fix now" button can drive it. */
  async tick() {
    if (this._busy) return null;
    this._busy = true;
    this.stats.attempts++;
    let pose = null;
    try {
      const image = await this._capture();
      const result = await this._localize(image);
      pose = this._toPose(result);
      if (pose) {
        this.stats.fixes++;
        this.stats.lastFixAt = Date.now();
        this._bus.publish(this._topic, { x: pose.x, y: pose.y, theta: pose.theta });
        this._onFix(pose, result);
      } else {
        this.stats.misses++;
        this._onMiss(new Error('localize answer unusable (few inliers / malformed)'));
      }
    } catch (err) {
      this.stats.misses++;
      this.stats.lastError = err.message || String(err);
      this._onMiss(err);
    } finally {
      this._busy = false;
      this._schedule(this._periodMs);
    }
    return pose;
  }
}
