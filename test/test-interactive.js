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
  before(function() {
    this.timeout(60 * 1000);
    return rclnodejs.init();
  });

  after(function() {
    rclnodejs.shutdown();
  });

  describe('Publisher/Subscription', function() {
    it('Publisher/Subscription', function(done) {
      var node = rclnodejs.createNode('publisher_subscription');
      var RclString = rclnodejs.require('std_msgs').msg.String;
      var msg = new RclString();
      var subscription = node.createSubscription(RclString, 'topic', function(msg) {
        assert.deepStrictEqual(typeof msg, 'object');
        assert.ok('data' in msg);
        assert.deepStrictEqual(msg.data, 'Greeting from publisher');

        node.destroy();
        done();
      });      
      rclnodejs.spin(node);

      childProcess.spawn('node', ['publisher_setup.js'], {cwd: __dirname});
    });
  });

  describe('Client/Service', function() {
    it('Client/Service', function(done) {
      var node = rclnodejs.createNode('client_service');
      var AddTwoInts = rclnodejs.require('example_interfaces').srv.AddTwoInts;
      var service = node.createService(AddTwoInts, 'add_two_ints', function(request, response) {
        assert.ok('a' in request);
        assert.deepStrictEqual(typeof request.a, 'number');
        assert.ok('b' in request);
        assert.deepStrictEqual(typeof request.b, 'number');
        response.sum = request.a + request.b;
        return response;
      });
      rclnodejs.spin(node);

      var clientProcess = childProcess.spawn('node', ['client_setup.js'], {cwd: __dirname});
      clientProcess.stdout.on('data', function(data) {
        node.destroy();
        assert.deepEqual(parseInt(data, 10), 3);
        done();
      });
    });
  });
});
