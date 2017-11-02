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
      const RclString = rclnodejs.require('std_msgs').msg.String;

      var cppReceived = false, pyReceived = false;
      var cppSubPath = path.join(process.env['AMENT_PREFIX_PATH'], 'lib', 'demo_nodes_cpp', 'listener');
      var cppSubscription = childProcess.spawn(cppSubPath, ['-t', 'js_pycpp_chatter']);
      var pySubPath = path.join(__dirname, 'py', 'listener.py');
      var pySubscription = utils.launchPythonProcess([pySubPath, 'js_pycpp_chatter']);

      let msg = new RclString();
      msg.data = 'hello world';
      var jsPublisher = node.createPublisher(RclString, 'js_pycpp_chatter');
      var timer = node.createTimer(100, () => {
        jsPublisher.publish(msg);
      });

      cppSubscription.stdout.on('data', (data) => {
        if (!cppReceived) {
          assert.ok(new RegExp('hello world').test(data.toString()));
          cppReceived = true;
          cppSubscription.kill('SIGINT');

          if (pyReceived) {
            timer.cancel();
            node.destroy();
            done();
          }
        }
      });

      pySubscription.stdout.on('data', (data) => {
        if (!pyReceived) {
          assert.ok(new RegExp('hello world').test(data.toString()));
          pyReceived = true;
          pySubscription.kill('SIGINT');

          if (cppReceived) {
            timer.cancel();
            node.destroy();
            done();
          }
        }
      });
      rclnodejs.spin(node);
    });

    it('Node.js subscription - Python publisher and Cpp publisher', function(done) {
      var node = rclnodejs.createNode('multi_nodes_js_subscription');
      const RclString = rclnodejs.require('std_msgs').msg.String;

      var receivedFromPy = false, receivedFromCpp = false;
      var subscription = node.createSubscription(RclString, 'pycpp_js_chatter', (msg) => {
        if (!receivedFromCpp) {
          if (new RegExp('Hello World:').test(msg.data)) {
            receivedFromCpp = true;
            cppPublisher.kill('SIGINT');

            if (receivedFromPy) {
              node.destroy();
              done();              
            }
          }          
        }

        if (!receivedFromPy) {
          if (new RegExp('Hello World').test(msg.data)) {
            receivedFromPy = true;
            pyPublisher.kill('SIGINT');

            if (receivedFromCpp) {
              node.destroy();
              done();
            }
          }
        }
      });

      var cppPubPath = path.join(process.env['AMENT_PREFIX_PATH'], 'lib', 'demo_nodes_cpp', 'talker');
      var cppPublisher = childProcess.spawn(cppPubPath, ['-t', 'pycpp_js_chatter']);
      var pyPubPath = path.join(__dirname, 'py', 'talker.py');
      var pyPublisher = utils.launchPythonProcess([pyPubPath, 'pycpp_js_chatter']);

      rclnodejs.spin(node);
    });
  });

  describe('Client/Service', function() {
    it('Node.js client - Cpp service and Python service', function(done) {
      var node = rclnodejs.createNode('multi_nodes_js_client');
      const AddTwoInts = rclnodejs.require('example_interfaces').srv.AddTwoInts;
      let request1 = new AddTwoInts.Request();
      request1.a = 1;
      request1.b = 2;
      let request2 = new AddTwoInts.Request();
      request2.a = 3;
      request2.b = 4;

      var cppServicePath = path.join(process.env['AMENT_PREFIX_PATH'],
                                    'lib',
                                    'demo_nodes_cpp',
                                    'add_two_ints_server');
      var cppService = childProcess.spawn(cppServicePath, ['-s', 'js_pycpp_add_two_ints']);
      var pyServicePath = path.join(__dirname, 'py', 'service.py');
      var pyService = utils.launchPythonProcess([pyServicePath, 'js_pycpp_add_two_ints']);

      var count = 1;
      var reachToCppService = false, reachToPyService = false;
      var client = node.createClient(AddTwoInts, 'js_pycpp_add_two_ints');
      var timer = node.createTimer(100, () => {
        if (count % 2) {
          client.sendRequest(request1, (response) => {
            if (!reachToCppService) {
              assert.deepStrictEqual(response.sum, 3);
              cppService.kill('SIGINT');
              reachToCppService = true;
            }
          });
        } else {
          client.sendRequest(request2, (response) => {
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
      const AddTwoInts = rclnodejs.require('example_interfaces').srv.AddTwoInts;
      var service = node.createService(AddTwoInts, 'pycpp_js_add_two_ints', (request, response) => {
        assert.deepStrictEqual(typeof request.a, 'number');
        assert.deepStrictEqual(typeof request.b, 'number');
        response.sum = request.a + request.b;
        return response;
      });
      rclnodejs.spin(node);
      var cppClientPath = path.join(process.env['AMENT_PREFIX_PATH'],
                                    'lib',
                                    'demo_nodes_cpp',
                                    'add_two_ints_client');
      var cppClient = childProcess.spawn(cppClientPath, ['-s', 'pycpp_js_add_two_ints']);
      var pyClientPath = path.join(__dirname, 'py', 'client.py');

      cppClient.stdout.on('data', (data) => {
        assert.deepStrictEqual(data.toString().trim(), 'Result of add_two_ints: 5');
        var pyClient = utils.launchPythonProcess([pyClientPath, 'pycpp_js_add_two_ints']);

        pyClient.stdout.on('data', (data) => {
          assert.deepStrictEqual(data.toString().trim(), '3');
          done();
        });
      });
    });
  });
});
