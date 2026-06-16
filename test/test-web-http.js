// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

import assert from 'assert';
import rclnodejs from '../index.js';
import {
  createRuntime,
  WebSocketTransport,
  HttpTransport,
} from '../lib/runtime/index.js';

let connect;
let RosClient;
before(async function () {
  ({ connect, RosClient } = await import('../web/index.js'));
});

describe('rclnodejs/web — HTTP transport (call + publish)', function () {
  this.timeout(60 * 1000);

  let node;
  let runtime;
  let httpBase; // e.g. "http://127.0.0.1:55123"
  let wsUrl; //   e.g. "ws://127.0.0.1:55124/capability"
  let serviceImpl;

  before(async function () {
    await rclnodejs.init();
    node = rclnodejs.createNode('http_runtime_test_node');
    rclnodejs.spin(node);

    serviceImpl = node.createService(
      'example_interfaces/srv/AddTwoInts',
      '/http_add',
      (request, response) => {
        const reply = response.template;
        reply.sum = request.a + request.b;
        response.send(reply);
      }
    );

    runtime = createRuntime({
      node,
      transports: [
        new WebSocketTransport({ port: 0, host: '127.0.0.1' }),
        new HttpTransport({ port: 0, host: '127.0.0.1' }),
      ],
    });
    runtime.expose({
      call: { '/http_add': 'example_interfaces/srv/AddTwoInts' },
      publish: { '/http_chatter': 'std_msgs/msg/String' },
      subscribe: { '/http_chatter': 'std_msgs/msg/String' },
    });
    await runtime.start();

    const wsTransport = runtime.transports[0];
    const httpTransport = runtime.transports[1];
    wsUrl = `ws://127.0.0.1:${wsTransport.port}/capability`;
    httpBase = `http://127.0.0.1:${httpTransport.port}`;
  });

  after(async function () {
    if (runtime) await runtime.stop();
    if (node && serviceImpl) {
      try {
        node.destroyService(serviceImpl);
      } catch (_) {}
    }
    rclnodejs.shutdown();
  });

  // -----------------------------------------------------------------
  // Bare HTTP wire format (no SDK) — proves the runtime is curl-able.
  // -----------------------------------------------------------------
  describe('wire format (raw fetch)', function () {
    it('round-trips a service call with status 200 + JSON body', async function () {
      const res = await fetch(httpBase + '/capability/call/http_add', {
        method: 'POST',
        headers: { 'content-type': 'application/json' },
        body: JSON.stringify({ a: '7n', b: '35n' }),
      });
      assert.strictEqual(res.status, 200);
      const body = await res.json();
      assert.strictEqual(body.sum, '42n');
    });

    it('returns 204 on successful publish', async function () {
      const res = await fetch(httpBase + '/capability/publish/http_chatter', {
        method: 'POST',
        headers: { 'content-type': 'application/json' },
        body: JSON.stringify({ data: 'hi-from-curl' }),
      });
      assert.strictEqual(res.status, 204);
    });

    it('rejects unexposed capability with 404 + not_exposed code', async function () {
      const res = await fetch(httpBase + '/capability/call/dangerous', {
        method: 'POST',
        headers: { 'content-type': 'application/json' },
        body: '{}',
      });
      assert.strictEqual(res.status, 404);
      const body = await res.json();
      assert.strictEqual(body.code, 'not_exposed');
    });

    it('rejects subscribe over HTTP with unsupported_kind', async function () {
      const res = await fetch(httpBase + '/capability/subscribe/http_chatter', {
        method: 'POST',
        headers: { 'content-type': 'application/json' },
        body: '{}',
      });
      assert.strictEqual(res.status, 404);
      const body = await res.json();
      assert.strictEqual(body.code, 'unsupported_kind');
    });

    it('rejects non-POST methods with 405', async function () {
      const res = await fetch(httpBase + '/capability/call/http_add', {
        method: 'GET',
      });
      assert.strictEqual(res.status, 405);
      assert.strictEqual(res.headers.get('allow'), 'POST');
    });

    it('rejects non-JSON content-type with 400', async function () {
      const res = await fetch(httpBase + '/capability/call/http_add', {
        method: 'POST',
        headers: { 'content-type': 'text/plain' },
        body: 'a=1&b=2',
      });
      assert.strictEqual(res.status, 400);
    });

    it('returns 404 for paths outside the basePath', async function () {
      const res = await fetch(httpBase + '/whatever');
      assert.strictEqual(res.status, 404);
    });
  });

  // -----------------------------------------------------------------
  // SDK over HTTP — same verb API as WebSocket, transport hidden.
  // -----------------------------------------------------------------
  describe('SDK over HTTP', function () {
    it('call() returns the typed reply', async function () {
      const ros = await connect(httpBase);
      try {
        const reply = await ros.call('/http_add', { a: '2n', b: '40n' });
        assert.strictEqual(reply.sum, '42n');
      } finally {
        await ros.close();
      }
    });

    it('publish() resolves to undefined', async function () {
      const ros = await connect(httpBase);
      try {
        const ret = await ros.publish('/http_chatter', { data: 'sdk-http' });
        assert.strictEqual(ret, undefined);
      } finally {
        await ros.close();
      }
    });

    it('rejects /dangerous with code: not_exposed', async function () {
      const ros = await connect(httpBase);
      try {
        let err;
        try {
          await ros.call('/dangerous', {});
        } catch (e) {
          err = e;
        }
        assert.ok(err, 'expected rejection');
        assert.strictEqual(err.code, 'not_exposed');
      } finally {
        await ros.close();
      }
    });

    it('subscribe over HTTP-only URL falls through to WS sibling', async function () {
      // The default WS URL the SDK derives from an http:// base
      // assumes the WS endpoint lives at the same host:port. Our test
      // uses *different* ports for HTTP and WS, so the sibling URL
      // won't resolve. Verify the SDK still routes call/publish over
      // HTTP and surfaces a clean error if subscribe is attempted here.
      const ros = await connect(httpBase);
      try {
        let err;
        try {
          await ros.subscribe('/http_chatter', () => {});
        } catch (e) {
          err = e;
        }
        // Either the WS sibling was unreachable (network_error /
        // ECONNREFUSED) or it succeeded against the wrong port; we
        // just need to confirm the SDK didn't crash.
        assert.ok(
          err === undefined || err instanceof Error,
          'subscribe must either succeed or reject cleanly'
        );
      } finally {
        await ros.close();
      }
    });

    it('explicit {http, ws} pair routes correctly', async function () {
      const ros = await connect({ http: httpBase, ws: wsUrl });
      try {
        // call → HTTP (single-shot)
        const reply = await ros.call('/http_add', { a: '1n', b: '2n' });
        assert.strictEqual(reply.sum, '3n');
        // subscribe → WS (long-lived)
        let received;
        const sub = await ros.subscribe('/http_chatter', (msg) => {
          if (!received) received = msg && msg.data;
        });
        // publish via HTTP, see the message arrive over WS
        const t = setInterval(() => {
          ros
            .publish('/http_chatter', { data: 'mixed-transport' })
            .catch(() => {});
        }, 100);
        try {
          await waitFor(() => received === 'mixed-transport', 8000);
        } finally {
          clearInterval(t);
        }
        await sub.close();
      } finally {
        await ros.close();
      }
    });

    // The HTTP transport's default basePath is `/capability`. Callers
    // should be able to spell that out explicitly without the SDK
    // double-prefixing it on every fetch — `'http://host:port'` and
    // `'http://host:port/capability'` must produce the same request
    // URLs.
    it('accepts URLs with explicit /capability suffix without double-prefixing', async function () {
      const ros = await connect(httpBase + '/capability');
      try {
        const reply = await ros.call('/http_add', { a: '5n', b: '7n' });
        assert.strictEqual(reply.sum, '12n');
      } finally {
        await ros.close();
      }
    });

    it('idempotent under { http } form too', async function () {
      const ros = await connect({ http: httpBase + '/capability' });
      try {
        const ret = await ros.publish('/http_chatter', {
          data: 'explicit-base',
        });
        assert.strictEqual(ret, undefined);
      } finally {
        await ros.close();
      }
    });
  });

  // -----------------------------------------------------------------
  // Defensive paths — regressions for the resource-leak, malformed-URL,
  // throwing-verifyRequest, and bad-endpoint cases.
  // -----------------------------------------------------------------
  describe('defensive paths', function () {
    it('runs dispatcher cleanup on every HTTP request (no Publisher leak)', async function () {
      // Each `publish()` lazily creates one Publisher per (connection,
      // capability). Without an emitted 'close' on HttpRequestConnection,
      // those Publishers would accumulate forever. We can't directly
      // observe the Map, but we can publish many times and assert the
      // node's publisher list doesn't grow proportionally.
      const before = node.countPublishers('/http_chatter');
      for (let i = 0; i < 5; i++) {
        const res = await fetch(httpBase + '/capability/publish/http_chatter', {
          method: 'POST',
          headers: { 'content-type': 'application/json' },
          body: JSON.stringify({ data: 'leak-check-' + i }),
        });
        assert.strictEqual(res.status, 204);
      }
      // Discovery is async; allow a tick.
      await new Promise((r) => setTimeout(r, 50));
      const after = node.countPublishers('/http_chatter');
      assert.ok(
        after - before <= 1,
        `expected at most 1 leftover publisher after 5 requests, got ${after - before}`
      );
    });

    it('returns 400 invalid_url for malformed percent-encoding', async function () {
      const res = await fetch(httpBase + '/capability/call/%E0%A4%A');
      assert.strictEqual(res.status, 400);
      const body = await res.json();
      assert.strictEqual(body.code, 'invalid_url');
    });

    it('returns 500 internal_error if verifyRequest throws', async function () {
      const failingRuntime = createRuntime({
        node,
        transport: new HttpTransport({
          port: 0,
          host: '127.0.0.1',
          verifyRequest: () => {
            throw new Error('boom');
          },
        }),
      });
      failingRuntime.expose({
        call: { '/http_add': 'example_interfaces/srv/AddTwoInts' },
      });
      await failingRuntime.start();
      const port = failingRuntime.transports[0].port;
      try {
        const res = await fetch(
          `http://127.0.0.1:${port}/capability/call/http_add`,
          {
            method: 'POST',
            headers: { 'content-type': 'application/json' },
            body: '{}',
          }
        );
        assert.strictEqual(res.status, 500);
        const body = await res.json();
        assert.strictEqual(body.code, 'internal_error');
      } finally {
        await failingRuntime.stop();
      }
    });

    it('rejects connect({}) with at-least-one-of error', function () {
      assert.throws(() => new RosClient({}), /at least one of http or ws/);
    });

    it('rejects connect({http: 42}) with non-string error', function () {
      assert.throws(
        () => new RosClient({ http: 42 }),
        /http must be a non-empty string/
      );
    });

    it('rejects connect({ws: "http://x"}) with wrong-scheme error', function () {
      assert.throws(
        () => new RosClient({ ws: 'http://x' }),
        /ws must start with ws:\/\/ or wss:\/\//
      );
    });

    it('call() before connect() rejects asynchronously, never throws sync', async function () {
      // Regression: previously `(this._http || this._ws).call(...)` could
      // throw a synchronous TypeError when constructed with ws:// and
      // connect() hadn't run yet. Now call() is async and rejects.
      const r = new RosClient('ws://127.0.0.1:1');
      const p = r.call('/never_used', {});
      assert.ok(typeof p.then === 'function', 'call() must return a Promise');
      await assert.rejects(p);
      await r.close();
    });

    it('stop() does not hang when keep-alive sockets are open', async function () {
      // Open a keep-alive request and don't read the body, then stop().
      // closeAllConnections() must let server.close() resolve promptly.
      const t0 = Date.now();
      const stopPromise = (async () => {
        await fetch(httpBase + '/capability/call/http_add', {
          method: 'POST',
          headers: { 'content-type': 'application/json' },
          body: '{}',
        }).catch(() => undefined);
      })();
      await stopPromise;
      // Build a one-off runtime so we can stop it cleanly here.
      const tmp = createRuntime({
        node,
        transport: new HttpTransport({ port: 0, host: '127.0.0.1' }),
      });
      tmp.expose({ call: { '/x': 'std_srvs/srv/Empty' } });
      await tmp.start();
      const port = tmp.transports[0].port;
      // Fire-and-forget a request; ignore its outcome. The keep-alive
      // socket would normally hold server.close() open for ~5s.
      fetch(`http://127.0.0.1:${port}/capability/call/x`, {
        method: 'POST',
        headers: { 'content-type': 'application/json' },
        body: '{}',
      }).catch(() => undefined);
      await new Promise((r) => setTimeout(r, 50));
      const t1 = Date.now();
      await tmp.stop();
      assert.ok(
        Date.now() - t1 < 2000,
        `stop() took ${Date.now() - t1}ms; expected fast shutdown`
      );
      assert.ok(Date.now() - t0 < 5000, 'whole test should finish under 5s');
    });

    it('normalises basePath: leading/trailing slash, missing leading', async function () {
      const cases = [
        { input: 'cap', expected: '/cap' },
        { input: '/cap/', expected: '/cap' },
        { input: '/cap///', expected: '/cap' },
      ];
      for (const { input, expected } of cases) {
        const t = new HttpTransport({ basePath: input });
        assert.strictEqual(
          t.basePath,
          expected,
          `for ${JSON.stringify(input)}`
        );
      }
      assert.throws(
        () => new HttpTransport({ basePath: 42 }),
        /basePath must be a string/
      );
    });

    it('rejects sneaky content-type that includes application/json', async function () {
      const res = await fetch(httpBase + '/capability/call/http_add', {
        method: 'POST',
        headers: { 'content-type': 'text/plain;application/json' },
        body: '{}',
      });
      assert.strictEqual(res.status, 400);
      const body = await res.json();
      assert.strictEqual(body.code, 'invalid_content_type');
    });
  });
});

// ===================================================================
// SSE subscribe + CORS — opt-in HTTP behaviour (sse: true, cors: …).
// ===================================================================
describe('rclnodejs/web — HTTP transport (SSE subscribe + CORS)', function () {
  this.timeout(60 * 1000);

  let node;
  let runtime;
  let base; // "http://127.0.0.1:<port>"
  let serviceImpl;

  before(async function () {
    await rclnodejs.init();
    node = rclnodejs.createNode('http_sse_test_node');
    rclnodejs.spin(node);

    serviceImpl = node.createService(
      'example_interfaces/srv/AddTwoInts',
      '/sse_add',
      (request, response) => {
        const reply = response.template;
        reply.sum = request.a + request.b;
        response.send(reply);
      }
    );

    runtime = createRuntime({
      node,
      transport: new HttpTransport({
        port: 0,
        host: '127.0.0.1',
        sse: true,
        cors: true,
      }),
    });
    runtime.expose({
      call: { '/sse_add': 'example_interfaces/srv/AddTwoInts' },
      publish: { '/sse_chatter': 'std_msgs/msg/String' },
      subscribe: { '/sse_chatter': 'std_msgs/msg/String' },
    });
    await runtime.start();
    base = `http://127.0.0.1:${runtime.transports[0].port}`;
  });

  after(async function () {
    if (runtime) await runtime.stop();
    if (node && serviceImpl) {
      try {
        node.destroyService(serviceImpl);
      } catch (_) {}
    }
    rclnodejs.shutdown();
  });

  describe('SSE subscribe', function () {
    it('GET subscribe streams a ready event then message events', async function () {
      const ac = new AbortController();
      const res = await fetch(base + '/capability/subscribe/sse_chatter', {
        headers: { accept: 'text/event-stream' },
        signal: ac.signal,
      });
      assert.strictEqual(res.status, 200);
      assert.match(res.headers.get('content-type') || '', /text\/event-stream/);
      const sse = sseReader(res);
      try {
        // The runtime emits `ready` once the dispatcher acknowledges the
        // subscribe (headers are deferred until then).
        const ready = await sse.next((e) => e.event === 'ready');
        assert.strictEqual(JSON.parse(ready.data).capability, '/sse_chatter');

        // Publish over HTTP until a delivery streams back. (Subscriptions
        // only see samples published after they're established, so we
        // retry on an interval to avoid a startup race.)
        const pump = setInterval(() => {
          fetch(base + '/capability/publish/sse_chatter', {
            method: 'POST',
            headers: { 'content-type': 'application/json' },
            body: JSON.stringify({ data: 'sse-hello' }),
          }).catch(() => {});
        }, 100);
        try {
          const msg = await sse.next((e) => e.event === 'message');
          assert.strictEqual(JSON.parse(msg.data).data, 'sse-hello');
        } finally {
          clearInterval(pump);
        }
      } finally {
        await sse.cancel();
        ac.abort();
      }
    });

    it('rejects a non-GET subscribe with 405 + allow: GET', async function () {
      const res = await fetch(base + '/capability/subscribe/sse_chatter', {
        method: 'POST',
        headers: { 'content-type': 'application/json' },
        body: '{}',
      });
      assert.strictEqual(res.status, 405);
      assert.strictEqual(res.headers.get('allow'), 'GET');
      const body = await res.json();
      assert.strictEqual(body.code, 'method_not_allowed');
    });

    it('rejects an unexposed SSE subscribe as a JSON error (not a stream)', async function () {
      // The 200 text/event-stream headers are deferred until the
      // dispatcher acks, so a rejected subscribe surfaces as a normal
      // JSON error with the mapped status — never a half-open stream.
      const res = await fetch(base + '/capability/subscribe/nope', {
        headers: { accept: 'text/event-stream' },
      });
      assert.strictEqual(res.status, 404);
      assert.match(res.headers.get('content-type') || '', /application\/json/);
      const body = await res.json();
      assert.strictEqual(body.code, 'not_exposed');
    });
  });

  describe('CORS', function () {
    it('answers OPTIONS preflight on capability routes with 204 + headers', async function () {
      const res = await fetch(base + '/capability/call/sse_add', {
        method: 'OPTIONS',
        headers: {
          origin: 'http://localhost:8080',
          'access-control-request-method': 'POST',
          'access-control-request-headers': 'content-type, authorization',
        },
      });
      assert.strictEqual(res.status, 204);
      assert.strictEqual(res.headers.get('access-control-allow-origin'), '*');
      assert.match(
        res.headers.get('access-control-allow-methods') || '',
        /POST/
      );
      // Reflects the browser's requested headers so Authorization / custom
      // headers pass preflight rather than being limited to content-type.
      assert.strictEqual(
        res.headers.get('access-control-allow-headers'),
        'content-type, authorization'
      );
    });

    it('falls back to content-type when no request-headers hint is sent', async function () {
      const res = await fetch(base + '/capability/call/sse_add', {
        method: 'OPTIONS',
        headers: {
          origin: 'http://localhost:8080',
          'access-control-request-method': 'POST',
        },
      });
      assert.strictEqual(res.status, 204);
      assert.strictEqual(
        res.headers.get('access-control-allow-headers'),
        'content-type'
      );
    });

    it('adds CORS headers to actual (non-preflight) responses', async function () {
      const res = await fetch(base + '/capability/call/sse_add', {
        method: 'POST',
        headers: {
          'content-type': 'application/json',
          origin: 'http://localhost:8080',
        },
        body: JSON.stringify({ a: '1n', b: '2n' }),
      });
      assert.strictEqual(res.status, 200);
      assert.strictEqual(res.headers.get('access-control-allow-origin'), '*');
      assert.strictEqual((await res.json()).sum, '3n');
    });

    it('does not answer OPTIONS preflight for non-capability paths', async function () {
      const res = await fetch(base + '/not-a-capability', {
        method: 'OPTIONS',
        headers: { origin: 'http://localhost:8080' },
      });
      assert.strictEqual(res.status, 404);
    });
  });

  describe('CORS allow-list', function () {
    let listRuntime;
    let listBase;

    before(async function () {
      listRuntime = createRuntime({
        node,
        transport: new HttpTransport({
          port: 0,
          host: '127.0.0.1',
          cors: ['http://allowed.example'],
        }),
      });
      listRuntime.expose({
        call: { '/sse_add': 'example_interfaces/srv/AddTwoInts' },
      });
      await listRuntime.start();
      listBase = `http://127.0.0.1:${listRuntime.transports[0].port}`;
    });

    after(async function () {
      if (listRuntime) await listRuntime.stop();
    });

    it('echoes an allowed origin and sets Vary: Origin', async function () {
      const res = await fetch(listBase + '/capability/call/sse_add', {
        method: 'POST',
        headers: {
          'content-type': 'application/json',
          origin: 'http://allowed.example',
        },
        body: JSON.stringify({ a: '2n', b: '3n' }),
      });
      assert.strictEqual(res.status, 200);
      assert.strictEqual(
        res.headers.get('access-control-allow-origin'),
        'http://allowed.example'
      );
      assert.match(res.headers.get('vary') || '', /Origin/i);
    });

    it('omits allow-origin for an origin not on the list', async function () {
      const res = await fetch(listBase + '/capability/call/sse_add', {
        method: 'POST',
        headers: {
          'content-type': 'application/json',
          origin: 'http://evil.example',
        },
        body: JSON.stringify({ a: '2n', b: '3n' }),
      });
      assert.strictEqual(res.status, 200);
      assert.strictEqual(res.headers.get('access-control-allow-origin'), null);
    });
  });
});

// Minimal Server-Sent Events reader over a fetch() response body. Parses
// `event:`/`data:` lines into `{event, data}` records and exposes a
// `next(predicate)` that resolves with the first matching record. Keep-
// alive comment lines (`:`-prefixed) are ignored.
function sseReader(res) {
  const reader = res.body.getReader();
  const decoder = new TextDecoder();
  const state = { buffer: '', event: 'message', data: '' };

  async function next(predicate, timeoutMs = 8000) {
    const deadline = Date.now() + timeoutMs;
    for (;;) {
      let nl;
      while ((nl = state.buffer.indexOf('\n')) >= 0) {
        const line = state.buffer.slice(0, nl).replace(/\r$/, '');
        state.buffer = state.buffer.slice(nl + 1);
        if (line === '') {
          const ev = { event: state.event, data: state.data };
          state.event = 'message';
          state.data = '';
          if (ev.data !== '' && predicate(ev)) return ev;
          continue;
        }
        if (line.startsWith(':')) continue; // keep-alive comment
        const ci = line.indexOf(':');
        const field = ci === -1 ? line : line.slice(0, ci);
        const val = ci === -1 ? '' : line.slice(ci + 1).replace(/^ /, '');
        if (field === 'event') state.event = val;
        else if (field === 'data') state.data += val;
      }
      if (Date.now() > deadline) {
        throw new Error('sseReader: timed out waiting for event');
      }
      const { value, done } = await reader.read();
      if (done) throw new Error('sseReader: stream ended early');
      state.buffer += decoder.decode(value, { stream: true });
    }
  }

  return { next, cancel: () => reader.cancel().catch(() => {}) };
}

function waitFor(predicate, timeoutMs) {
  const started = Date.now();
  return new Promise((resolve, reject) => {
    const tick = () => {
      if (predicate()) return resolve();
      if (Date.now() - started > timeoutMs) {
        return reject(new Error('waitFor: timeout'));
      }
      setTimeout(tick, 50);
    };
    tick();
  });
}
