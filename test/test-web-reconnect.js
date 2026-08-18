// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// Reconnect-on-close coverage for `rclnodejs/web`'s Browser SDK WebSocket
// link. See test/test-web-ws.js / test/test-web-http.js for the base
// transport tests this builds on.

import assert from 'assert';
import http from 'node:http';
import rclnodejs from '../index.js';
import { createRuntime, WebSocketTransport } from '../lib/runtime/index.js';

let connect;
let RosClient;
before(async function () {
  ({ connect, RosClient } = await import('../web/index.js'));
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
      // A *new* request after the drop hits the _closed guard, not the
      // in-flight connection_lost path — see the test below for that case.
      await assertRejectsWithCode(
        ros.call('/rc_add_slow', { a: '1n', b: '2n' }),
        undefined
      );
    } finally {
      await ros.close();
    }
  });

  it('without reconnect: true, an in-flight call() still gets a connection_lost code', async function () {
    const ros = await connect(endpoint);
    try {
      const pending = ros.call('/rc_add_slow', { a: '1n', b: '2n' });
      await dropServerSideConnection();
      await assertRejectsWithCode(pending, 'connection_lost');
    } finally {
      await ros.close();
    }
  });

  it('the first connection attempt rejects once and never starts reconnecting, even if error precedes close', async function () {
    // A port nothing listens on: the underlying socket fails via 'error'
    // followed by 'close' — the same ordering a mid-session drop uses, but
    // on the very first attempt, which must reject once and stop there.
    const deadPort = await new Promise((resolve) => {
      const srv = http.createServer();
      srv.listen(0, '127.0.0.1', () => {
        const { port } = srv.address();
        srv.close(() => resolve(port));
      });
    });

    const ros = new RosClient(`ws://127.0.0.1:${deadPort}/capability`, {
      reconnect: true,
    });
    let reconnecting = false;
    ros.on('reconnecting', () => {
      reconnecting = true;
    });

    await assert.rejects(ros.connect());
    await new Promise((resolve) => setTimeout(resolve, 800));
    assert.strictEqual(reconnecting, false);
  });

  it('close() during backoff finalizes cleanly instead of staying in a reconnecting state', async function () {
    const ros = await connect(endpoint, { reconnect: true });
    try {
      await dropServerSideConnection();
      // Still mid-backoff: the base delay is >=250ms, and the dropped
      // socket's readyState is already 3 (CLOSED) at this point. Without
      // the fix, _isReconnecting never resets and this next call() would
      // reject with connection_lost forever instead of a definitive close.
      await new Promise((resolve) => setTimeout(resolve, 50));
      await ros.close();
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
