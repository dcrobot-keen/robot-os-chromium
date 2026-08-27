// LocalBus — topic-namespaced pub/sub over BroadcastChannel (architecture
// doc, Layer 05, "local" half). Available as a global in both browsers and
// Node 18+, so this same class works in a page, a Web Worker, or a Node
// test script.
//
// A BroadcastChannel instance never delivers a message back to the same
// instance that posted it — only *other* BroadcastChannel objects on the
// same channel name receive it, including ones created in the same page.
// That's surprising if you expect ordinary pub/sub: a node that both
// publishes and subscribes on the same LocalBus instance (the common case
// — see apps/dashboard/index.html) would never see its own messages.
// publish() below works around this by dispatching to this instance's own
// subscribers directly, in addition to broadcasting — so publish/subscribe
// behaves the way it looks like it should regardless of whether the
// publisher and subscriber happen to share an instance.

export class LocalBus {
  constructor(channelName = 'ros-chromium') {
    this._channel = new BroadcastChannel(channelName);
    this._handlers = new Map(); // topic -> Set<cb>
    this._channel.onmessage = (event) => this._dispatch(event.data.topic, event.data.payload);
  }

  publish(topic, payload) {
    this._channel.postMessage({ topic, payload });
    this._dispatch(topic, payload); // see the class-level note: BroadcastChannel won't do this part itself
  }

  // Returns an unsubscribe function.
  subscribe(topic, cb) {
    if (!this._handlers.has(topic)) this._handlers.set(topic, new Set());
    this._handlers.get(topic).add(cb);
    return () => this._handlers.get(topic)?.delete(cb);
  }

  _dispatch(topic, payload) {
    const cbs = this._handlers.get(topic);
    if (cbs) for (const cb of cbs) cb(payload);
  }

  close() {
    this._channel.close();
  }
}
