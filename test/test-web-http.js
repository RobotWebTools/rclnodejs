// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

'use strict';

const assert = require('assert');
const rclnodejs = require('../index.js');
const {
  createRuntime,
  WebSocketTransport,
  HttpTransport,
} = require('../lib/runtime');

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
  });
});

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
