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

describe('rclnodejs interactive testing', function() {
  this.timeout(60 * 1000);

  before(function() {
    return rclnodejs.init();
  });

  after(function() {
    rclnodejs.shutdown();
  });

  describe('Publisher/Subscription', function() {
    it('Publisher/Subscription', function(done) {
      var node = rclnodejs.createNode('publisher_subscription');
      const RclString = 'std_msgs/msg/String';
      var publisher = childProcess.fork(`${__dirname}/publisher_setup.js`);
      var destroy = false;
      var subscription = node.createSubscription(RclString, 'topic', function(msg) {
        assert.deepStrictEqual(typeof msg, 'object');
        assert.ok('data' in msg);
        assert.deepStrictEqual(msg.data, 'Greeting from publisher');

        if (!destroy) {
          publisher.kill('SIGINT');
          node.destroy();
          destroy = true;
          done();
        }
      });      
      rclnodejs.spin(node);
    });
  });

  describe('Client/Service', function() {
    it('Client/Service', function(done) {
      var node = rclnodejs.createNode('client_service');
      var AddTwoInts = 'example_interfaces/srv/AddTwoInts';
      var service = node.createService(AddTwoInts, 'add_two_ints', function(request, response) {
        assert.ok('a' in request);
        assert.deepStrictEqual(typeof request.a, 'number');
        assert.ok('b' in request);
        assert.deepStrictEqual(typeof request.b, 'number');
        let result = response.template;
        result.sum = request.a + request.b;
        return result;
      });
      rclnodejs.spin(node);

      var destroy = false;
      var client = childProcess.fork(`${__dirname}/client_setup.js`, {silent: true});
      client.stdout.on('data', function(data) {
        assert.ok(new RegExp('3').test(data.toString()));
        if (!destroy) {
          client.kill('SIGINT');
          node.destroy();
          destroy = true;
          done();
        }
      });
    });
  });
});
