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

describe('Test rclnodejs nodes in a single process', function() {
  this.timeout(60 * 1000);

  before(function() {
    return rclnodejs.init();
  });

  after(function() {
    rclnodejs.shutdown();
  });

  it('Publisher/Subscription in one process', function(done) {
    var publisherNode = rclnodejs.createNode('single_ps_publisher');
    var subscriptionNode = rclnodejs.createNode('single_ps_subscription');
    const RclString = rclnodejs.require('std_msgs').msg.String;
    let msg = new RclString();
    msg.data = 'Hello World';

    var subscription = subscriptionNode.createSubscription(RclString, 'single_ps_channel1', (msg) => {
      timer.cancel();
      assert.deepStrictEqual(msg.data, 'Hello World');
      done();
    });

    var publisher = publisherNode.createPublisher(RclString, 'single_ps_channel1');
    var timer = publisherNode.createTimer(100, () => {
      publisher.publish(msg);
    });
    rclnodejs.spin(subscriptionNode);
    rclnodejs.spin(publisherNode);
  });

  it('New style requiring for messages', function(done) {
    var node = rclnodejs.createNode('new_style_require_message');
    const RclString = rclnodejs.require('std_msgs/msg/String');
    let msg = new RclString();
    msg.data = 'Hello World';

    var subscription = node.createSubscription(RclString, 'new_style_require1', (msg) => {
      timer.cancel();
      assert.deepStrictEqual(msg.data, 'Hello World');
      done();
    }); 

    var publisher = node.createPublisher(RclString, 'new_style_require1');
    var timer = node.createTimer(100, () => {
      publisher.publish(msg);
    });

    rclnodejs.spin(node);
  });

  it('Client/Service is a one process', function(done) {
    var clientNode = rclnodejs.createNode('single_ps_client');
    var serviceNode = rclnodejs.createNode('single_ps_service');
    const AddTwoInts = 'example_interfaces/srv/AddTwoInts';

    var service = serviceNode.createService(AddTwoInts, 'single_ps_channel2', (request, response) => {
      assert.deepStrictEqual(request.a, 1);
      assert.deepStrictEqual(request.b, 2);
      response.sum = request.a + request.b;
      return response;
    });
    var client = clientNode.createClient(AddTwoInts, 'single_ps_channel2');
    let request = {a: 1, b: 2};

    var timer = clientNode.createTimer(100, () => {
      client.sendRequest(request, (response) => {
        timer.cancel();
        assert.deepStrictEqual(response.sum, 3);
        done();
      });
    });
    rclnodejs.spin(serviceNode);
    rclnodejs.spin(clientNode);
  });

  it('New style requiring for services', function(done) {
    var node = rclnodejs.createNode('new_style_require_services');
    const AddTwoInts = rclnodejs.require('example_interfaces/srv/AddTwoInts');

    var service = node.createService(AddTwoInts, 'new_style_require2', (request, response) => {
      assert.deepStrictEqual(request.a, 1);
      assert.deepStrictEqual(request.b, 2);
      response.sum = request.a + request.b;
      return response;
    });
    var client = node.createClient(AddTwoInts, 'new_style_require2');
    let request = new AddTwoInts.Request();
    request.a = 1;
    request.b = 2;

    var timer = node.createTimer(100, () => {
      client.sendRequest(request, (response) => {
        timer.cancel();
        assert.deepStrictEqual(response.sum, 3);
        done();
      });
    });

    rclnodejs.spin(node);
  });
});
