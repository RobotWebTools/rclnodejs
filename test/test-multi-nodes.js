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
const path = require('path');
const childProcess = require('child_process');
const rclnodejs = require('../index.js');
const utils = require('./utils.js');
const kill = require('tree-kill');

describe('Multiple nodes interation testing', function() {
  this.timeout(60 * 1000);

  before(function() {
    return rclnodejs.init();
  });

  after(function() {
    rclnodejs.shutdown();
  });

  describe('Publisher/Subscription', function() {
    it('Node.js publisher - Python and Cpp subscription', function(done) {
      var node = rclnodejs.createNode('multi_nodes_js_publisher');
      const RclString = 'std_msgs/msg/String';

      var cppReceived = false,
        pyReceived = false;
      var cppSubPath = path.join(__dirname, 'cpp', 'listener');
      var cppSubscription = childProcess.spawn(cppSubPath, [
        '-t',
        'js_pycpp_chatter',
      ]);
      var pySubPath = path.join(__dirname, 'py', 'listener.py');
      var pySubscription = utils.launchPythonProcess([
        pySubPath,
        'js_pycpp_chatter',
      ]);

      const msg = 'hello world';
      var count = 0;
      var jsSubscription1 = node.createSubscription(
        RclString,
        'back_js_pycpp_chatter',
        backMsg => {
          count++;
          assert.deepStrictEqual(backMsg.data, msg);

          if (count === 2) {
            node.destroy();
            cppSubscription.kill('SIGINT');
            pySubscription.kill('SIGINT');
            done();
          }
        }
      );
      var jsPublisher = node.createPublisher(RclString, 'js_pycpp_chatter');
      setTimeout(() => {
        jsPublisher.publish(msg);
      }, 1000);
      rclnodejs.spin(node);
    });

    it('Node.js subscription - Python publisher and Cpp publisher', function(done) {
      var node = rclnodejs.createNode('multi_nodes_js_subscription');
      const RclString = 'std_msgs/msg/String';

      var receivedFromPy = false,
        receivedFromCpp = false;
      var subscription = node.createSubscription(RclString, 'chatter', msg => {
        if (!receivedFromCpp) {
          if (new RegExp('Hello World:').test(msg.data)) {
            receivedFromCpp = true;
            kill(cppPublisher.pid, 'SIGINT');

            if (receivedFromPy) {
              node.destroy();
              done();
            }
          }
        }

        if (!receivedFromPy) {
          if (new RegExp('Hello World').test(msg.data)) {
            receivedFromPy = true;
            kill(pyPublisher.pid, 'SIGINT');

            if (receivedFromCpp) {
              node.destroy();
              done();
            }
          }
        }
      });

      var cppPublisher = childProcess.spawn('ros2', [
        'run',
        'demo_nodes_cpp',
        'talker',
      ]);
      var pyPubPath = path.join(__dirname, 'py', 'talker.py');
      var pyPublisher = utils.launchPythonProcess([pyPubPath, 'chatter']);

      rclnodejs.spin(node);
    });
  });

  describe('Client/Service', function() {
    it('Node.js client - Cpp service and Python service', function(done) {
      var node = rclnodejs.createNode('multi_nodes_js_client');
      const AddTwoInts = 'example_interfaces/srv/AddTwoInts';
      const request1 = { a: 1, b: 2 };
      const request2 = { a: 3, b: 4 };

      var cppServicePath = path.join(
        utils.getAvailablePath(process.env['AMENT_PREFIX_PATH'], [
          'lib',
          'demo_nodes_cpp',
          'add_two_ints_server',
        ])
      );
      var cppService = childProcess.spawn(cppServicePath, [
        '-s',
        'js_pycpp_add_two_ints',
      ]);
      var pyServicePath = path.join(__dirname, 'py', 'service.py');
      var pyService = utils.launchPythonProcess([
        pyServicePath,
        'js_pycpp_add_two_ints',
      ]);

      var count = 1;
      var reachToCppService = false,
        reachToPyService = false;
      var client = node.createClient(AddTwoInts, 'js_pycpp_add_two_ints');
      var timer = node.createTimer(500, () => {
        if (count % 2) {
          client.sendRequest(request1, response => {
            if (!reachToCppService) {
              assert.deepStrictEqual(response.sum, 3);
              cppService.kill('SIGINT');
              reachToCppService = true;
            }
          });
        } else {
          client.sendRequest(request2, response => {
            if (!reachToPyService) {
              assert.deepStrictEqual(response.sum, 7);
              pyService.kill('SIGINT');
              reachToPyService = true;
            }
          });
        }
        count++;

        if (reachToCppService && reachToPyService) {
          timer.cancel();
          node.destroy();
          done();
        }
      });
      rclnodejs.spin(node);
    });

    it('Node.js service - Cpp client and Python client', function(done) {
      var node = rclnodejs.createNode('multi_nodes_js_service');
      const AddTwoInts = 'example_interfaces/srv/AddTwoInts';
      const Int8 = 'std_msgs/msg/Int8';
      var service = node.createService(
        AddTwoInts,
        'pycpp_js_add_two_ints',
        (request, response) => {
          assert.deepStrictEqual(typeof request.a, 'number');
          assert.deepStrictEqual(typeof request.b, 'number');
          let result = response.template;
          result.sum = request.a + request.b;
          response.send(result);
        }
      );

      var pyReceived = false,
        cppReceived = false;
      var cppSubscription = node.createSubscription(
        Int8,
        'back_pycpp_js_add_two_ints',
        backMsg => {
          if (backMsg.data === 5) cppReceived = true;

          if (backMsg.data === 3) pyReceived = true;

          if (cppReceived && pyReceived) {
            node.destroy();
            cppClient.kill('SIGINT');
            pyClient.kill('SIGINT');
            done();
          }
        }
      );
      rclnodejs.spin(node);

      var pyClientPath = path.join(__dirname, 'py', 'client.py');
      var cppClientPath = path.join(__dirname, 'cpp', 'add_two_ints_client');
      var pyClient, cppClient;
      setTimeout(() => {
        pyClient = utils.launchPythonProcess([
          pyClientPath,
          'pycpp_js_add_two_ints',
        ]);
        cppClient = childProcess.spawn(cppClientPath, [
          '-s',
          'pycpp_js_add_two_ints',
        ]);
      }, 1000);
    });
  });
});
