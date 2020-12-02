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
    const RclString = 'std_msgs/msg/String';
    const msg = 'Hello World';

    var subscription = subscriptionNode.createSubscription(
      RclString,
      'single_ps_channel1',
      msg => {
        timer.cancel();
        assert.deepStrictEqual(msg.data, 'Hello World');
        publisherNode.destroy();
        subscriptionNode.destroy();
        done();
      }
    );

    var publisher = publisherNode.createPublisher(
      RclString,
      'single_ps_channel1'
    );
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

    var subscription = node.createSubscription(
      RclString,
      'new_style_require1',
      msg => {
        timer.cancel();
        assert.deepStrictEqual(msg.data, 'Hello World');
        node.destroy();
        done();
      }
    );

    var publisher = node.createPublisher(RclString, 'new_style_require1');
    var timer = node.createTimer(100, () => {
      publisher.publish(msg);
    });

    rclnodejs.spin(node);
  });

  it('Client/Service in one process', function(done) {
    var clientNode = rclnodejs.createNode('single_ps_client');
    var serviceNode = rclnodejs.createNode('single_ps_service');
    const AddTwoInts = 'example_interfaces/srv/AddTwoInts';

    var service = serviceNode.createService(
      AddTwoInts,
      'single_ps_channel2',
      (request, response) => {
        assert.deepStrictEqual(request.a, 1);
        assert.deepStrictEqual(request.b, 2);
        let result = response.template;
        result.sum = request.a + request.b;
        return result;
      }
    );
    var client = clientNode.createClient(AddTwoInts, 'single_ps_channel2');
    const request = { a: 1, b: 2 };

    var timer = clientNode.createTimer(100, () => {
      client.sendRequest(request, response => {
        timer.cancel();
        assert.deepStrictEqual(response.sum, 3);
        serviceNode.destroy();
        clientNode.destroy();
        done();
      });
    });
    rclnodejs.spin(serviceNode);
    rclnodejs.spin(clientNode);
  });

  it('Client/Service in one process - service callback syntax #2', function(done) {
    var clientNode = rclnodejs.createNode('single_ps_client_2');
    var serviceNode = rclnodejs.createNode('single_ps_service_2');
    const AddTwoInts = 'example_interfaces/srv/AddTwoInts';

    var service = serviceNode.createService(
      AddTwoInts,
      'single_ps_channel2_2',
      (request, response) => {
        assert.deepStrictEqual(request.a, 1);
        assert.deepStrictEqual(request.b, 2);
        let result = response.template;
        result.sum = request.a + request.b;
        response.send(result);
      }
    );
    var client = clientNode.createClient(AddTwoInts, 'single_ps_channel2_2');
    const request = { a: 1, b: 2 };

    var timer = clientNode.createTimer(100, () => {
      client.sendRequest(request, response => {
        timer.cancel();
        assert.deepStrictEqual(response.sum, 3);
        serviceNode.destroy();
        clientNode.destroy();
        done();
      });
    });
    rclnodejs.spin(serviceNode);
    rclnodejs.spin(clientNode);
  });

  it('Client/Service in one process - service callback syntax #3', function(done) {
    var clientNode = rclnodejs.createNode('single_ps_client_3');
    var serviceNode = rclnodejs.createNode('single_ps_service_3');
    const AddTwoInts = 'example_interfaces/srv/AddTwoInts';

    var service = serviceNode.createService(
      AddTwoInts,
      'single_ps_channel2_3',
      (request, response) => {
        assert.deepStrictEqual(request.a, 1);
        assert.deepStrictEqual(request.b, 2);
        response.send({ sum: request.a + request.b });
      }
    );
    var client = clientNode.createClient(AddTwoInts, 'single_ps_channel2_3');
    const request = { a: 1, b: 2 };

    var timer = clientNode.createTimer(100, () => {
      client.sendRequest(request, response => {
        timer.cancel();
        assert.deepStrictEqual(response.sum, 3);
        serviceNode.destroy();
        clientNode.destroy();
        done();
      });
    });
    rclnodejs.spin(serviceNode);
    rclnodejs.spin(clientNode);
  });

  it('New style requiring for services', function(done) {
    var node = rclnodejs.createNode('new_style_require_services');
    const AddTwoInts = rclnodejs.require('example_interfaces/srv/AddTwoInts');

    var service = node.createService(
      AddTwoInts,
      'new_style_require2',
      (request, response) => {
        assert.deepStrictEqual(request.a, 1);
        assert.deepStrictEqual(request.b, 2);
        let result = response.template;
        result.sum = request.a + request.b;
        return result;
      }
    );
    var client = node.createClient(AddTwoInts, 'new_style_require2');
    let request = new AddTwoInts.Request();
    request.a = 1;
    request.b = 2;

    var timer = node.createTimer(100, () => {
      client.sendRequest(request, response => {
        timer.cancel();
        assert.deepStrictEqual(response.sum, 3);
        node.destroy();
        done();
      });
    });

    rclnodejs.spin(node);
  });
});
