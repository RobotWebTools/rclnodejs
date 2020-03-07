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
const assertUtils = require('./utils.js');
const assertThrowsError = assertUtils.assertThrowsError;

describe('Node extra destroy methods testing', function() {
  before(function() {
    return rclnodejs.init();
  });

  after(function() {
    rclnodejs.shutdown();
  });

  it('destroyPublisher()', function() {
    var node = rclnodejs.createNode('node1');
    const RclString = 'std_msgs/msg/String';
    // const RclString = rclnodejs.require('std_msgs/msg/String');
    var publisher = node.createPublisher(RclString, 'chatter');
    assert.deepStrictEqual(node._publishers.length, 2);

    assertThrowsError(
      function() {
        node.destroyPublisher('publisher');
      },
      TypeError,
      'Invalid argument',
      'Invalid type of parameter'
    );

    node.destroyPublisher(publisher);
    assert.deepStrictEqual(node._publishers.length, 1);
  });

  it('destroySubscription()', function() {
    var node = rclnodejs.createNode('node2');
    const RclString = 'std_msgs/msg/String';
    // const RclString = rclnodejs.require('std_msgs/msg/String');
    var subscription = node.createSubscription(RclString, 'chatter', () => {});
    assert.deepStrictEqual(node._subscriptions.length, 1);

    assertThrowsError(
      function() {
        node.destroySubscription('subscription');
      },
      TypeError,
      'Invalid argument',
      'Invalid type of parameter'
    );

    node.destroySubscription(subscription);
    assert.deepStrictEqual(node._subscriptions.length, 0);
  });

  it('destroyClient()', function() {
    var node = rclnodejs.createNode('node3');
    const AddTwoInts = 'example_interfaces/srv/AddTwoInts';
    // const AddTwoInts = rclnodejs.require('example_interfaces/srv/AddTwoInts');
    var client = node.createClient(AddTwoInts, 'add_two_ints');
    assert.deepStrictEqual(node._clients.length, 1);

    assertThrowsError(
      function() {
        node.destroyClient('client');
      },
      TypeError,
      'Invalid argument',
      'Invalid type of parameter'
    );

    node.destroyClient(client);
    assert.deepStrictEqual(node._clients.length, 0);
  });

  it('destroyService()', function() {
    var node = rclnodejs.createNode('node4');
    const AddTwoInts = 'example_interfaces/srv/AddTwoInts';
    // const AddTwoInts = rclnodejs.require('example_interfaces/srv/AddTwoInts');
    var service = node.createService(AddTwoInts, 'add_two_ints', () => {});
    assert.deepStrictEqual(node._services.length, 6);

    assertThrowsError(
      function() {
        node.destroyService('service');
      },
      TypeError,
      'Invalid argument',
      'Invalid type of parameter'
    );

    node.destroyService(service);
    assert.deepStrictEqual(node._services.length, 5);
  });

  it('destroyTimer()', function() {
    var node = rclnodejs.createNode('node5');
    var timer = node.createTimer(1000, () => {});
    assert.deepStrictEqual(node._timers.length, 1);

    assertThrowsError(
      function() {
        node.destroyTimer('timer');
      },
      TypeError,
      'Invalid argument',
      'Invalid type of parameter'
    );

    node.destroyTimer(timer);
    assert.deepStrictEqual(node._timers.length, 0);
  });
});
