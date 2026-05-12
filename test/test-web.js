// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

'use strict';

// SDK tests for `rclnodejs/web`. Runtime / wire protocol coverage
// lives in test/test-runtime.js.

const assert = require('assert');
const rclnodejs = require('../index.js');
const { createRuntime, WebSocketTransport } = require('../lib/runtime');

// `web/` is ESM; await import() lets us pull it in from this CJS file.
let connect;
before(async function () {
  ({ connect } = await import('../web/index.js'));
});

describe('rclnodejs/web (Browser SDK)', function () {
  this.timeout(60 * 1000);

  let node;
  let runtime;
  let endpoint;

  before(async function () {
    await rclnodejs.init();
    node = rclnodejs.createNode('runtime_test_node');
    rclnodejs.spin(node);

    runtime = createRuntime({
      node,
      transport: new WebSocketTransport({ port: 0, host: '127.0.0.1' }),
    });
    runtime.expose({
      call: { '/rt_add': 'example_interfaces/srv/AddTwoInts' },
      publish: { '/rt_chatter': 'std_msgs/msg/String' },
      subscribe: { '/rt_chatter': 'std_msgs/msg/String' },
    });
    await runtime.start();
    const port = runtime.transports[0].port;
    endpoint = `ws://127.0.0.1:${port}/capability`;
  });

  after(async function () {
    if (runtime) await runtime.stop();
    rclnodejs.shutdown();
  });

  it('round-trips a service call through the runtime', async function () {
    const svc = node.createService(
      'example_interfaces/srv/AddTwoInts',
      '/rt_add',
      (request, response) => {
        const reply = response.template;
        reply.sum = request.a + request.b;
        response.send(reply);
      }
    );
    const ros = await connect(endpoint);
    try {
      // BigInts arrive over the wire as the toJSONSafe "Nn" convention.
      const reply = await ros.call('/rt_add', { a: '2n', b: '40n' });
      assert.strictEqual(reply.sum, '42n');
    } finally {
      await ros.close();
      node.destroyService(svc);
    }
  });

  it('publish + subscribe round-trip through the runtime', async function () {
    const ros = await connect(endpoint);
    let timer;
    try {
      let received;
      const sub = await ros.subscribe('/rt_chatter', (msg) => {
        if (!received) received = msg && msg.data;
      });

      // Retry until subscription discovery completes.
      timer = setInterval(() => {
        ros.publish('/rt_chatter', { data: 'hello-runtime' }).catch(() => {});
      }, 100);

      await waitFor(() => received === 'hello-runtime', 8000);
      await sub.close();
    } finally {
      if (timer) clearInterval(timer);
      await ros.close();
    }
  });

  it('surfaces not_exposed errors with a structured `code` field', async function () {
    // Asserts the SDK propagates `code:` onto the rejected Promise.
    const ros = await connect(endpoint);
    try {
      await assertRejectsWithCode(
        ros.call('/never_exposed', {}),
        'not_exposed'
      );
      await assertRejectsWithCode(
        ros.publish('/never_exposed', {}),
        'not_exposed'
      );
      await assertRejectsWithCode(
        ros.subscribe('/never_exposed', () => {}),
        'not_exposed'
      );
    } finally {
      await ros.close();
    }
  });

  it('assigns distinct UUID v4 ids for repeated subscribes', async function () {
    const ros = await connect(endpoint);
    try {
      const sub1 = await ros.subscribe('/rt_chatter', () => {});
      const sub2 = await ros.subscribe('/rt_chatter', () => {});
      assert.notStrictEqual(sub1.subId, sub2.subId);
      assert.match(
        sub1.subId,
        /^[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[0-9a-f]{4}-[0-9a-f]{12}$/i,
        'subId should be a v4 UUID'
      );
      await sub1.close();
      await sub2.close();
    } finally {
      await ros.close();
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
