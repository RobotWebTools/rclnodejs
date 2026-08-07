// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// Reconnect-on-close (WebSocket) and retry-with-backoff (HTTP) coverage
// for `rclnodejs/web`'s Browser SDK. See test/test-web-ws.js /
// test/test-web-http.js for the base transport tests this builds on.

import assert from 'assert';
import http from 'node:http';
import rclnodejs from '../index.js';
import { createRuntime, WebSocketTransport } from '../lib/runtime/index.js';

let connect;
before(async function () {
  ({ connect } = await import('../web/index.js'));
});

describe('rclnodejs/web — WebSocket reconnect', function () {
  this.timeout(60 * 1000);

  let node;
  let runtime;
  let endpoint;
  let wss;

  before(async function () {
    await rclnodejs.init();
    node = rclnodejs.createNode('reconnect_test_node');
    rclnodejs.spin(node);

    // Deliberately slow so a test can drop the connection while a call()
    // is still in flight and observe the pending rejection.
    node.createService(
      'example_interfaces/srv/AddTwoInts',
      '/rc_add_slow',
      (request, response) => {
        setTimeout(() => {
          const reply = response.template;
          reply.sum = request.a + request.b;
          response.send(reply);
        }, 300);
      }
    );

    runtime = createRuntime({
      node,
      transport: new WebSocketTransport({ port: 0, host: '127.0.0.1' }),
    });
    runtime.expose({
      call: { '/rc_add_slow': 'example_interfaces/srv/AddTwoInts' },
      publish: { '/rc_chatter': 'std_msgs/msg/String' },
      subscribe: { '/rc_chatter': 'std_msgs/msg/String' },
    });
    await runtime.start();
    const port = runtime.transports[0].port;
    endpoint = `ws://127.0.0.1:${port}/capability`;
    wss = runtime.transports[0]._wss;
  });

  after(async function () {
    if (runtime) await runtime.stop();
    rclnodejs.shutdown();
  });

  // Forcibly terminate the server-side socket for the (single) currently
  // connected client, simulating an abrupt network drop rather than a
  // graceful close.
  async function dropServerSideConnection() {
    await waitFor(() => wss.clients.size === 1, 5000);
    const [serverSocket] = wss.clients;
    serverSocket.terminate();
  }

  it('reconnects after an unexpected drop and replays subscriptions', async function () {
    const ros = await connect(endpoint, { reconnect: true });
    const events = [];
    ros.on('disconnected', () => events.push('disconnected'));
    ros.on('reconnecting', (detail) => events.push({ reconnecting: detail }));
    ros.on('reconnected', () => events.push('reconnected'));

    try {
      let received;
      await ros.subscribe('/rc_chatter', (msg) => {
        received = msg && msg.data;
      });

      await dropServerSideConnection();
      await waitFor(() => events.includes('reconnected'), 15000);

      // 'disconnected' must fire before any 'reconnecting', which must
      // fire before 'reconnected'.
      const disconnectedAt = events.indexOf('disconnected');
      const reconnectedAt = events.indexOf('reconnected');
      assert.ok(disconnectedAt >= 0 && disconnectedAt < reconnectedAt);
      assert.ok(events.some((e) => e && e.reconnecting));

      // The original subscription (never re-created by the test) must
      // still be receiving messages after the transparent resubscribe.
      received = undefined;
      const timer = setInterval(() => {
        ros.publish('/rc_chatter', { data: 'after-reconnect' }).catch(() => {});
      }, 100);
      try {
        await waitFor(() => received === 'after-reconnect', 8000);
      } finally {
        clearInterval(timer);
      }
    } finally {
      await ros.close();
    }
  });

  it('rejects an in-flight call() with connection_lost when the socket drops', async function () {
    const ros = await connect(endpoint, { reconnect: true });
    try {
      const pending = ros.call('/rc_add_slow', { a: '1n', b: '2n' });
      await dropServerSideConnection();
      await assertRejectsWithCode(pending, 'connection_lost');
    } finally {
      await ros.close();
    }
  });

  it('rejects new requests with connection_lost while a reconnect is pending', async function () {
    const ros = await connect(endpoint, { reconnect: true });
    try {
      let disconnected = false;
      ros.on('disconnected', () => {
        disconnected = true;
      });
      await dropServerSideConnection();
      await waitFor(() => disconnected, 5000);
      // The client has processed the drop, but the backoff timer
      // (>=250ms) hasn't fired yet, so no reconnect attempt has started.
      await assertRejectsWithCode(
        ros.call('/rc_add_slow', { a: '1n', b: '2n' }),
        'connection_lost'
      );
    } finally {
      await ros.close();
    }
  });

  it('without reconnect: true, a drop is final (unchanged default behavior)', async function () {
    const ros = await connect(endpoint);
    try {
      let disconnected = false;
      ros.on('disconnected', () => {
        disconnected = true;
      });
      await dropServerSideConnection();
      await waitFor(() => disconnected, 5000);
      await assertRejectsWithCode(
        ros.call('/rc_add_slow', { a: '1n', b: '2n' }),
        undefined
      );
    } finally {
      await ros.close();
    }
  });

  it('ros.close() never triggers a reconnect attempt', async function () {
    const ros = await connect(endpoint, { reconnect: true });
    let reconnecting = false;
    ros.on('reconnecting', () => {
      reconnecting = true;
    });
    await ros.close();
    await new Promise((resolve) => setTimeout(resolve, 800));
    assert.strictEqual(reconnecting, false);
  });
});

describe('rclnodejs/web — HTTP retry-with-backoff', function () {
  this.timeout(60 * 1000);

  function startMockServer(handler) {
    const server = http.createServer(handler);
    return new Promise((resolve) => {
      server.listen(0, '127.0.0.1', () => resolve(server));
    });
  }

  async function stopServer(server) {
    await new Promise((resolve) => server.close(resolve));
  }

  it('retries on a 5xx and eventually succeeds', async function () {
    let attempts = 0;
    const server = await startMockServer((req, res) => {
      attempts++;
      if (attempts < 3) {
        res.writeHead(500, { 'content-type': 'application/json' });
        res.end(JSON.stringify({ ok: false, error: 'boom' }));
        return;
      }
      res.writeHead(200, { 'content-type': 'application/json' });
      res.end(JSON.stringify({ sum: '42n' }));
    });
    try {
      const ros = await connect(
        { http: `http://127.0.0.1:${server.address().port}` },
        { httpRetries: 3 }
      );
      try {
        const reply = await ros.call('/whatever', { a: '1n', b: '2n' });
        assert.strictEqual(reply.sum, '42n');
        assert.strictEqual(attempts, 3);
      } finally {
        await ros.close();
      }
    } finally {
      await stopServer(server);
    }
  });

  it('retries on a network error (connection reset) and eventually succeeds', async function () {
    let attempts = 0;
    const server = await startMockServer((req, res) => {
      attempts++;
      if (attempts === 1) {
        req.socket.destroy(); // simulate a mid-request network failure
        return;
      }
      res.writeHead(200, { 'content-type': 'application/json' });
      res.end(JSON.stringify({ sum: '42n' }));
    });
    try {
      const ros = await connect(
        { http: `http://127.0.0.1:${server.address().port}` },
        { httpRetries: 2 }
      );
      try {
        const reply = await ros.call('/whatever', { a: '1n', b: '2n' });
        assert.strictEqual(reply.sum, '42n');
        assert.strictEqual(attempts, 2);
      } finally {
        await ros.close();
      }
    } finally {
      await stopServer(server);
    }
  });

  it('gives up after httpRetries attempts and surfaces the last error', async function () {
    let attempts = 0;
    const server = await startMockServer((req, res) => {
      attempts++;
      res.writeHead(500, { 'content-type': 'application/json' });
      res.end(JSON.stringify({ ok: false, error: 'boom' }));
    });
    try {
      const ros = await connect(
        { http: `http://127.0.0.1:${server.address().port}` },
        { httpRetries: 2 }
      );
      try {
        await assertRejectsWithCode(
          ros.call('/whatever', { a: '1n', b: '2n' }),
          'http_500'
        );
        assert.strictEqual(attempts, 3); // 1 initial + 2 retries
      } finally {
        await ros.close();
      }
    } finally {
      await stopServer(server);
    }
  });

  it('does not retry a 4xx error even with httpRetries set', async function () {
    let attempts = 0;
    const server = await startMockServer((req, res) => {
      attempts++;
      res.writeHead(404, { 'content-type': 'application/json' });
      res.end(
        JSON.stringify({ ok: false, error: 'nope', code: 'not_exposed' })
      );
    });
    try {
      const ros = await connect(
        { http: `http://127.0.0.1:${server.address().port}` },
        { httpRetries: 3 }
      );
      try {
        await assertRejectsWithCode(
          ros.call('/whatever', { a: '1n', b: '2n' }),
          'not_exposed'
        );
        assert.strictEqual(attempts, 1);
      } finally {
        await ros.close();
      }
    } finally {
      await stopServer(server);
    }
  });

  it('default httpRetries: 0 makes exactly one attempt', async function () {
    let attempts = 0;
    const server = await startMockServer((req, res) => {
      attempts++;
      res.writeHead(500, { 'content-type': 'application/json' });
      res.end(JSON.stringify({ ok: false, error: 'boom' }));
    });
    try {
      const ros = await connect({
        http: `http://127.0.0.1:${server.address().port}`,
      });
      try {
        await assertRejectsWithCode(
          ros.call('/whatever', { a: '1n', b: '2n' }),
          'http_500'
        );
        assert.strictEqual(attempts, 1);
      } finally {
        await ros.close();
      }
    } finally {
      await stopServer(server);
    }
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

async function assertRejectsWithCode(promise, expectedCode) {
  let err;
  try {
    await promise;
  } catch (e) {
    err = e;
  }
  assert.ok(err, 'expected rejection');
  assert.strictEqual(
    err.code,
    expectedCode,
    `expected error code ${expectedCode}, got ${err.code}: ${err.message}`
  );
}
