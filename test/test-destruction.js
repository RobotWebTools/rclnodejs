// Copyright (c) 2017 Intel Corporation. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

'use strict';

const assert = require('assert');
const childProcess = require('child_process');
const rclnodejs = require('../index.js');

describe('Node & Entity destroy testing', function () {
  this.timeout(60 * 1000);

  before(function () {
    return rclnodejs.init();
  });

  after(function () {
    rclnodejs.shutdown();
  });

  it('node.destroy()', function () {
    var node = rclnodejs.createNode('my_node1');
    node.destroy();
  });

  it('node.destroy() twice', function () {
    var node = rclnodejs.createNode('my_node2');
    node.destroy();

    assert.doesNotThrow(function () {
      node.destroy();
    }, Error);
  });

  it('node handle readonly', function () {
    var node = rclnodejs.createNode('my_node3');
    try {
      node.handle = 'gargage';
      assert.fail('node.handle should be read-only');
    } catch (Error) {
      assert.ok(true);
      node.destroy();
    }
  });

  it('node.destroy entities', function () {
    var node = rclnodejs.createNode('my_node5');

    var defaultPublishersCount = node._publishers.length;
    var defaultServicesCount = node._services.length;

    // timers
    var timer1 = node.createTimer(0.1, () => {});
    var timer2 = node.createTimer(1, () => {});
    assert.deepStrictEqual(2, node._timers.length);
    node.destroyTimer(timer1);
    assert.deepStrictEqual(1, node._timers.length);

    // publishers
    const int16 = 'std_msgs/msg/Int16';
    var pub1 = node.createPublisher(int16, 'pub1_topic');
    assert.deepStrictEqual(defaultPublishersCount + 1, node._publishers.length);
    node.destroyPublisher(pub1);
    assert.ok(pub1.isDestroyed());
    assert.deepStrictEqual(defaultPublishersCount, node._publishers.length);

    const float32 = 'std_msgs/msg/Float32';
    var pub2 = node.createPublisher(float32, 'pub2_topic');
    assert.deepStrictEqual(defaultPublishersCount + 1, node._publishers.length);
    assert.ok(!pub2.isDestroyed());

    // subscriptions
    var RclString = 'std_msgs/msg/String';
    var sub1 = node.createSubscription(
      RclString,
      'sub1_topic',
      function (msg) {}
    );
    assert.deepStrictEqual(1, node._subscriptions.length);
    node.destroySubscription(sub1);
    assert.ok(sub1.isDestroyed());
    assert.deepStrictEqual(0, node._subscriptions.length);

    const uint8 = 'std_msgs/msg/UInt8';
    var sub2 = node.createSubscription(uint8, 'sub2_topic', function (msg) {});
    assert.ok(!sub2.isDestroyed());
    assert.deepStrictEqual(1, node._subscriptions.length);

    // clients
    const srvInterface = 'example_interfaces/srv/AddTwoInts';
    const serviceName = 'add_two_ints';
    var client = node.createClient(srvInterface, serviceName);
    assert.ok(!client.isDestroyed());
    assert.deepStrictEqual(1, node._clients.length);
    node.destroyClient(client);
    assert.ok(client.isDestroyed());
    assert.deepStrictEqual(0, node._clients.length);
    client = node.createClient(srvInterface, serviceName);

    // services
    var service = node.createService(srvInterface, serviceName, () => {});
    assert.ok(!service.isDestroyed());
    assert.deepStrictEqual(defaultServicesCount + 1, node._services.length);
    node.destroyService(service);
    assert.ok(service.isDestroyed());
    assert.deepStrictEqual(defaultServicesCount, node._services.length);
    service = node.createService(srvInterface, serviceName, () => {});

    node.destroy();
    assert.deepStrictEqual(0, node._timers.length);
    assert.deepStrictEqual(0, node._publishers.length);
    assert.deepStrictEqual(0, node._subscriptions.length);
    assert.deepStrictEqual(0, node._clients.length);
    assert.deepStrictEqual(0, node._services.length);
  });

  it('entity redundant destroy()', function (done) {
    var node = rclnodejs.createNode('my_node7');

    // timers
    var timer = node.createTimer(0.1, () => {});
    node.destroyTimer(timer);
    node.destroyTimer(timer);

    // publishers
    const int16 = 'std_msgs/msg/Int16';
    var pub1 = node.createPublisher(int16, 'pub1_topic');
    node.destroyPublisher(pub1);
    node.destroyPublisher(pub1);

    // subscriptions
    var RclString = 'std_msgs/msg/String';
    var sub1 = node.createSubscription(
      RclString,
      'sub1_topic',
      function (msg) {}
    );
    node.destroySubscription(sub1);
    node.destroySubscription(sub1);

    // clients
    const srvInterface = 'example_interfaces/srv/AddTwoInts';
    const serviceName = 'add_two_ints';
    var client = node.createClient(srvInterface, serviceName);
    node.destroyClient(client);
    node.destroyClient(client);

    // services
    var service = node.createService(srvInterface, serviceName, () => {});
    node.destroyService(service);
    node.destroyService(service);

    done();
  });

  it('rapid subscription create & destroy', function (done) {
    const node = rclnodejs.createNode('node8');
    const msgString = 'std_msgs/msg/String';
    const publisher = childProcess.fork(`${__dirname}/publisher_msg.js`, [
      'String',
      '"RCL String"',
    ]);
    let subscription1 = node.createSubscription(
      msgString,
      'String_channel',
      (msg) => {}
    );

    let fail = false;
    try {
      let subscription2 = node.createSubscription(
        msgString,
        'String_channel',
        (msg) => {
          node.destroySubscription(subscription1);
          subscription1 = node.createSubscription(
            msgString,
            'String_channel',
            (msg) => {}
          );
        }
      );
    } catch (e) {
      fail = true;
    }

    setTimeout(() => {
      publisher.kill('SIGINT');
      assert.ok(!fail);
      done();
    }, 1500);

    rclnodejs.spin(node);
  });
});
