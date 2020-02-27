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
const rclnodejs = require('../index.js');

describe('Node destroy testing', function() {
  this.timeout(60 * 1000);

  before(function() {
    return rclnodejs.init();
  });

  after(function() {
    rclnodejs.shutdown();
  });

  it('node.destroy()', function() {
    var node = rclnodejs.createNode('my_node1');
    node.destroy();
  });

  it('node.destroy() twice', function() {
    var node = rclnodejs.createNode('my_node1');
    node.destroy();

    assert.doesNotThrow(function() {
      node.destroy();
    }, Error);
  });

  it('node.destroy() corrupted', function() {
    var node = rclnodejs.createNode('my_node3');
    node.destroy();

    var orgHandle = node.handle;
    node.handle = 'garbage';
    try {
      node.destroy();
    } catch (TypeError) {
      assert.ok(true);
      node.handle = orgHandle;
      node.destroy();
    }
  });

  it('node.destory timers', function() {
    var node = rclnodejs.createNode('my_node4');
    var timer1 = node.createTimer(0.1, () => {}),
      timer2 = node.createTimer(1, () => {});

    assert.deepStrictEqual(2, node._timers.length);
    node.destroy();
    assert.deepStrictEqual(0, node._timers.length);
  });

  it('node destory entities', function() {
    var node = rclnodejs.createNode('my_node5');

    var timer = node.createTimer(0.1, () => {});
    assert.deepStrictEqual(1, node._timers.length);

    const int16 = 'std_msgs/msg/Int16';
    var pub1 = node.createPublisher(int16, 'pub1_topic');
    assert.deepStrictEqual(2, node._publishers.length);

    const float32 = 'std_msgs/msg/Float32';
    var pub2 = node.createPublisher(float32, 'pub2_topic');
    assert.deepStrictEqual(3, node._publishers.length);

    var RclString = 'std_msgs/msg/String';
    var sub1 = node.createSubscription(RclString, 'sub1_topic', function(msg) {
      console.log(`Received ${msg}`);
    });
    assert.deepStrictEqual(1, node._subscriptions.length);

    const uint8 = 'std_msgs/msg/UInt8';
    var sub2 = node.createSubscription(uint8, 'sub2_topic', function(msg) {
      console.log(`Received ${msg}`);
    });
    assert.deepStrictEqual(2, node._subscriptions.length);

    node.destroy();
    assert.deepStrictEqual(0, node._timers.length);
    assert.deepStrictEqual(0, node._publishers.length);
    assert.deepStrictEqual(0, node._subscriptions.length);
  });

  it('node currupted node handle', function() {
    var node = rclnodejs.createNode('my_node6');
    try {
      node.handle = 'gargage';
    } catch (Error) {
      assert.ok(true);
      node.destroy();
    }
  });
});
