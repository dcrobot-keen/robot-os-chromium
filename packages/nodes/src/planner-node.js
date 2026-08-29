// PlannerNode — roadmap.md Phase 7's "PlannerNode: 격자 A*(또는 Theta*) → path",
// backed by pathfinder's WASM build (@ros-chromium/planner-wasm) instead of a
// from-scratch implementation. Subscribes to a plan-request topic (an
// occupancy grid it does NOT build itself, plus start/goal/algorithm) and
// publishes { requestId, path, distance } or { requestId, error } on a
// separate path topic.
//
// Scope note: this is only the "given a grid, find a path" half of Phase 7.
// Building that grid from LIDAR scans (MapNode) and turning a path into
// cmd_vel (PathFollowerNode) are separate, not-yet-implemented nodes per
// roadmap.md -- this node doesn't assume either exists yet, which is why it
// takes a grid as part of the request instead of reading one off the bus.
import { loadPlanner } from '@ros-chromium/planner-wasm';

export class PlannerNode {
  constructor(bus, { requestTopic, pathTopic } = {}) {
    if (!requestTopic || !pathTopic) {
      throw new Error('PlannerNode requires requestTopic and pathTopic');
    }
    this._bus = bus;
    this._pathTopic = pathTopic;
    this._plannerPromise = loadPlanner();
    this._unsubscribe = bus.subscribe(requestTopic, (request) => this._handleRequest(request));
  }

  async _handleRequest({ requestId, ...findPathRequest }) {
    try {
      const planner = await this._plannerPromise;
      const { path, distance } = planner.findPath(findPathRequest);
      this._bus.publish(this._pathTopic, { requestId, path, distance });
    } catch (err) {
      this._bus.publish(this._pathTopic, { requestId, error: err.message });
    }
  }

  stop() {
    this._unsubscribe();
  }
}
