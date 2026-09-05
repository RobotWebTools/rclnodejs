// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

import assert from 'assert';
import WebSocket from 'ws';
import rclnodejs from '../index.js';
import {
  createRuntime,
  CapabilityRegistry,
  WebSocketTransport,
} from '../lib/runtime/index.js';

describe('CapabilityRegistry (unit)', function () {
  it('expose() registers shorthand and rich forms', function () {
    const reg = new CapabilityRegistry();
    reg.expose({
      call: { '/add': 'example_interfaces/srv/AddTwoInts' },
      publish: { '/chatter': { type: 'std_msgs/msg/String' } },
      subscribe: { '/scan': 'sensor_msgs/msg/LaserScan' },
      action: { '/fibonacci': 'example_interfaces/action/Fibonacci' },
    });
    assert.deepStrictEqual(reg.list(), {
      call: { '/add': 'example_interfaces/srv/AddTwoInts' },
      publish: { '/chatter': 'std_msgs/msg/String' },
      subscribe: { '/scan': 'sensor_msgs/msg/LaserScan' },
      action: { '/fibonacci': 'example_interfaces/action/Fibonacci' },
    });
  });

  it('resolve() returns the matching capability or null', function () {
    const reg = new CapabilityRegistry().expose({
      call: { '/add': 'example_interfaces/srv/AddTwoInts' },
      action: { '/fibonacci': 'example_interfaces/action/Fibonacci' },
    });
    assert.deepStrictEqual(reg.resolve('call', '/add'), {
      kind: 'call',
      name: '/add',
      type: 'example_interfaces/srv/AddTwoInts',
    });
    assert.deepStrictEqual(reg.resolve('action', '/fibonacci'), {
      kind: 'action',
      name: '/fibonacci',
      type: 'example_interfaces/action/Fibonacci',
    });
    assert.strictEqual(reg.resolve('call', '/missing'), null);
    assert.strictEqual(reg.resolve('publish', '/add'), null);
    assert.strictEqual(reg.resolve('action', '/add'), null);
  });

  it('expose() rejects malformed values', function () {
    const reg = new CapabilityRegistry();
    assert.throws(() => reg.expose({ call: { '/x': '' } }), /empty string/);
    assert.throws(() => reg.expose({ call: { '/x': { type: '' } } }), /empty/);
    assert.throws(
      () => reg.expose({ call: { '/x': 42 } }),
      /string or { type: string }/
    );
    assert.throws(
      () => reg.expose({ call: { '/x': null } }),
      /string or { type: string }/
    );
  });
});

describe('Web Runtime end-to-end (WebSocket transport)', function () {
  this.timeout(60 * 1000);

  let node;
  let runtime;

  before(async function () {
    await rclnodejs.init();
    node = rclnodejs.createNode('runtime_test_node');
    rclnodejs.spin(node);
    runtime = createRuntime({
      node,
      transports: [new WebSocketTransport({ port: 0 })],
    });
    runtime.expose({
      call: { '/wb_add': 'example_interfaces/srv/AddTwoInts' },
      publish: { '/wb_pub_in': 'std_msgs/msg/String' },
      subscribe: { '/wb_chatter': 'std_msgs/msg/String' },
    });
    await runtime.start();
  });

  after(async function () {
    if (runtime) await runtime.stop();
    rclnodejs.shutdown();
  });

  function url() {
    return `ws://127.0.0.1:${runtime.transports[0].port}/capability`;
  }

  function waitOpen(ws) {
    return new Promise((resolve, reject) => {
      ws.once('open', resolve);
      ws.once('error', reject);
    });
  }

  // Wait for the next frame matching `predicate` and resolve with it.
  function waitFrame(ws, predicate) {
    return new Promise((resolve) => {
      const onMsg = (data) => {
        const frame = JSON.parse(data.toString('utf8'));
        if (predicate(frame)) {
          ws.off('message', onMsg);
          resolve(frame);
        }
      };
      ws.on('message', onMsg);
    });
  }

  it('rejects capabilities not in the allow-list with code:not_exposed', async function () {
    const ws = new WebSocket(url());
    await waitOpen(ws);
    const replyP = waitFrame(ws, (f) => f.id === 'r1');
    ws.send(
      JSON.stringify({
        id: 'r1',
        kind: 'call',
        capability: '/dangerous',
        payload: {},
      })
    );
    const reply = await replyP;
    assert.strictEqual(reply.ok, false);
    assert.strictEqual(reply.code, 'not_exposed');
    ws.close();
  });

  it('rejects unknown wire kinds with code:unknown_kind', async function () {
    const ws = new WebSocket(url());
    await waitOpen(ws);
    const replyP = waitFrame(ws, (f) => f.id === 'k1');
    ws.send(JSON.stringify({ id: 'k1', kind: 'mystery', capability: '/x' }));
    const reply = await replyP;
    assert.strictEqual(reply.ok, false);
    assert.strictEqual(reply.code, 'unknown_kind');
    ws.close();
  });

  it('rejects an action frame with a missing/unknown op with code:unknown_kind', async function () {
    const ws = new WebSocket(url());
    await waitOpen(ws);
    const replyP = waitFrame(ws, (f) => f.id === 'a1');
    ws.send(
      JSON.stringify({ id: 'a1', kind: 'action', capability: '/anything' })
    );
    const reply = await replyP;
    assert.strictEqual(reply.ok, false);
    assert.strictEqual(reply.code, 'unknown_kind');
    ws.close();
  });

  it('rejects action send_goal against an unexposed capability with code:not_exposed', async function () {
    const ws = new WebSocket(url());
    await waitOpen(ws);
    const replyP = waitFrame(ws, (f) => f.id === 'a2');
    ws.send(
      JSON.stringify({
        id: 'a2',
        kind: 'action',
        op: 'send_goal',
        capability: '/anything',
        payload: {},
      })
    );
    const reply = await replyP;
    assert.strictEqual(reply.ok, false);
    assert.strictEqual(reply.code, 'not_exposed');
    ws.close();
  });

  it('rejects non-JSON frames with code:invalid_json', async function () {
    const ws = new WebSocket(url());
    await waitOpen(ws);
    const replyP = waitFrame(ws, (f) => f.code === 'invalid_json');
    ws.send('this is not json');
    const reply = await replyP;
    assert.strictEqual(reply.ok, false);
    ws.close();
  });

  it('rejects binary frames with code:binary_unsupported', async function () {
    const ws = new WebSocket(url());
    await waitOpen(ws);
    const replyP = waitFrame(ws, (f) => f.code === 'binary_unsupported');
    ws.send(Buffer.from([0x01, 0x02, 0x03]));
    const reply = await replyP;
    assert.strictEqual(reply.ok, false);
    ws.close();
  });

  it('round-trips a service call against an exposed capability', async function () {
    // Service backing the registered capability.
    const svc = node.createService(
      'example_interfaces/srv/AddTwoInts',
      '/wb_add',
      (request, response) => {
        const reply = response.template;
        reply.sum = request.a + request.b;
        response.send(reply);
      }
    );

    try {
      const ws = new WebSocket(url());
      await waitOpen(ws);
      const replyP = waitFrame(ws, (f) => f.id === 'c1');
      ws.send(
        JSON.stringify({
          id: 'c1',
          kind: 'call',
          capability: '/wb_add',
          payload: { a: '2n', b: '40n' },
        })
      );
      const reply = await replyP;
      assert.strictEqual(reply.ok, true);
      assert.strictEqual(reply.payload.sum, '42n');
      ws.close();
    } finally {
      node.destroyService(svc);
    }
  });

  it('publishes from the browser into ROS via an exposed publish capability', async function () {
    const seen = [];
    const sub = node.createSubscription(
      'std_msgs/msg/String',
      '/wb_pub_in',
      (msg) => seen.push(msg.data)
    );

    try {
      const ws = new WebSocket(url());
      await waitOpen(ws);
      const replyP = waitFrame(ws, (f) => f.id === 'p1');
      ws.send(
        JSON.stringify({
          id: 'p1',
          kind: 'publish',
          capability: '/wb_pub_in',
          payload: { data: 'from-runtime' },
        })
      );
      const reply = await replyP;
      assert.strictEqual(reply.ok, true);

      // Wait for the subscription to observe the publication.
      const start = Date.now();
      while (!seen.includes('from-runtime') && Date.now() - start < 5000) {
        await new Promise((r) => setTimeout(r, 25));
      }
      assert.ok(
        seen.includes('from-runtime'),
        `expected to receive 'from-runtime', got ${JSON.stringify(seen)}`
      );
      ws.close();
    } finally {
      node.destroySubscription(sub);
    }
  });

  it('subscribe streams messages until the client unsubscribes', async function () {
    const pub = node.createPublisher('std_msgs/msg/String', '/wb_chatter');
    let timer;

    try {
      const ws = new WebSocket(url());
      await waitOpen(ws);

      // Open subscription with id 's1'.
      const ackP = waitFrame(ws, (f) => f.id === 's1' && f.ok === true);
      ws.send(
        JSON.stringify({
          id: 's1',
          kind: 'subscribe',
          capability: '/wb_chatter',
        })
      );
      await ackP;

      // Drive ROS publications until at least one delivery arrives.
      const deliveryP = waitFrame(
        ws,
        (f) => f.event === 'message' && f.subId === 's1'
      );
      timer = setInterval(() => pub.publish('hello-runtime'), 100);
      const delivery = await deliveryP;
      assert.strictEqual(delivery.payload.data, 'hello-runtime');

      // Now unsubscribe.
      const unsubAckP = waitFrame(ws, (f) => f.id === 's1-stop');
      ws.send(
        JSON.stringify({ id: 's1-stop', kind: 'unsubscribe', subId: 's1' })
      );
      const unsubAck = await unsubAckP;
      assert.strictEqual(unsubAck.ok, true);

      // A second unsubscribe must report unknown_sub_id.
      const errP = waitFrame(ws, (f) => f.id === 's1-stop2');
      ws.send(
        JSON.stringify({ id: 's1-stop2', kind: 'unsubscribe', subId: 's1' })
      );
      const err = await errP;
      assert.strictEqual(err.ok, false);
      assert.strictEqual(err.code, 'unknown_sub_id');

      ws.close();
    } finally {
      clearInterval(timer);
      node.destroyPublisher(pub);
    }
  });

  it('rejects subscribe with a duplicate id with code:duplicate_id', async function () {
    const ws = new WebSocket(url());
    await waitOpen(ws);

    const ackP = waitFrame(ws, (f) => f.id === 'dup' && f.ok === true);
    ws.send(
      JSON.stringify({
        id: 'dup',
        kind: 'subscribe',
        capability: '/wb_chatter',
      })
    );
    await ackP;

    const errP = waitFrame(ws, (f) => f.id === 'dup' && f.ok === false);
    ws.send(
      JSON.stringify({
        id: 'dup',
        kind: 'subscribe',
        capability: '/wb_chatter',
      })
    );
    const err = await errP;
    assert.strictEqual(err.code, 'duplicate_id');

    // Tear down via unsubscribe so the connection close cleanup path is exercised
    // by the after() hook in addition to the explicit one.
    ws.send(
      JSON.stringify({ id: 'dup-stop', kind: 'unsubscribe', subId: 'dup' })
    );
    ws.close();
  });

  it('createRuntime requires a node', function () {
    assert.throws(() => createRuntime({}), /node is required/);
  });
});
