// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

'use strict';

const assert = require('assert');
const WebSocket = require('ws');
const rclnodejs = require('../index.js');
const { startRosocket } = require('../rosocket');
const rosocket = require('../rosocket');

describe('rosocket resource-style WebSocket gateway', function () {
  this.timeout(60 * 1000);

  it('exports startRosocket', function () {
    assert.strictEqual(rosocket.startRosocket, startRosocket);
  });

  let node;
  let bridge;

  before(async function () {
    await rclnodejs.init();
    node = rclnodejs.createNode('rosocket_test_node');
    rclnodejs.spin(node);
    bridge = await startRosocket({
      node,
      port: 0, // ephemeral
      topicTypes: { '/wb_chatter': 'std_msgs/msg/String' },
      serviceTypes: { '/wb_add': 'example_interfaces/srv/AddTwoInts' },
    });
  });

  after(async function () {
    if (bridge) await bridge.close();
    rclnodejs.shutdown();
  });

  function url(path) {
    return `ws://127.0.0.1:${bridge.port}${path}`;
  }

  function waitOpen(ws) {
    return new Promise((resolve, reject) => {
      ws.once('open', resolve);
      ws.once('error', reject);
    });
  }

  it('rejects unknown paths', function (done) {
    const ws = new WebSocket(url('/bogus/foo'));
    let gotError = false;
    ws.on('message', (data) => {
      const msg = JSON.parse(data.toString());
      assert.ok(msg.error);
      gotError = true;
    });
    ws.on('close', () => {
      assert.ok(gotError, 'expected error frame before close');
      done();
    });
  });

  it('rejects topics without a type', function (done) {
    const ws = new WebSocket(url('/topic/no_type_topic'));
    let gotError = false;
    ws.on('message', (data) => {
      const msg = JSON.parse(data.toString());
      assert.match(msg.error, /Missing message type/);
      gotError = true;
    });
    ws.on('close', () => {
      assert.ok(gotError);
      done();
    });
  });

  it('subscribes via configured topicTypes default', function (done) {
    const ws = new WebSocket(url('/topic/wb_chatter'));
    const pub = node.createPublisher('std_msgs/msg/String', '/wb_chatter');
    let timer;
    ws.on('open', () => {
      timer = setInterval(() => pub.publish('hello-bridge'), 100);
    });
    ws.on('message', (data) => {
      const msg = JSON.parse(data.toString());
      if (msg && msg.data === 'hello-bridge') {
        clearInterval(timer);
        node.destroyPublisher(pub);
        ws.close();
        done();
      }
    });
    ws.on('error', done);
  });

  it('publishes from browser into ROS via ?type= query', function (done) {
    const sub = node.createSubscription(
      'std_msgs/msg/String',
      '/wb_pub_in',
      (msg) => {
        if (msg.data === 'from-ws') {
          node.destroySubscription(sub);
          done();
        }
      }
    );
    const ws = new WebSocket(url('/topic/wb_pub_in?type=std_msgs/msg/String'));
    waitOpen(ws)
      .then(() => ws.send(JSON.stringify({ data: 'from-ws' })))
      .catch(done);
  });

  it('round-trips a service call (bare request shape)', function (done) {
    const svc = node.createService(
      'example_interfaces/srv/AddTwoInts',
      '/wb_add',
      (request, response) => {
        const r = response.template;
        r.sum = request.a + request.b;
        response.send(r);
      }
    );
    const ws = new WebSocket(url('/service/wb_add'));
    // AddTwoInts uses int64; send as BigInt-encoded strings so the bridge revives them.
    waitOpen(ws)
      .then(() => ws.send(JSON.stringify({ a: '4n', b: '5n' })))
      .catch(done);
    ws.on('message', (data) => {
      const msg = JSON.parse(data.toString());
      assert.ok(!msg.error, `unexpected error frame: ${JSON.stringify(msg)}`);
      // sum is int64 -> rclnodejs toJSONSafe encodes as "9n"
      assert.strictEqual(msg.sum, '9n');
      node.destroyService(svc);
      ws.close();
      done();
    });
  });

  it('round-trips a service call with id wrapping', function (done) {
    const svc = node.createService(
      'example_interfaces/srv/AddTwoInts',
      '/wb_add_wrapped',
      (request, response) => {
        const r = response.template;
        r.sum = request.a + request.b;
        response.send(r);
      }
    );
    const ws = new WebSocket(
      url('/service/wb_add_wrapped?type=example_interfaces/srv/AddTwoInts')
    );
    waitOpen(ws)
      .then(() =>
        ws.send(JSON.stringify({ id: 'c1', request: { a: '7n', b: '8n' } }))
      )
      .catch(done);
    ws.on('message', (data) => {
      const msg = JSON.parse(data.toString());
      assert.ok(!msg.error, `unexpected error frame: ${JSON.stringify(msg)}`);
      assert.strictEqual(msg.id, 'c1');
      assert.ok(msg.response);
      assert.strictEqual(msg.response.sum, '15n');
      node.destroyService(svc);
      ws.close();
      done();
    });
  });
});
