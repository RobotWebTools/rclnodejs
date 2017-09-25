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

describe('Cross-language interaction', function() {
  describe('Node.js Subcription', function() {
    before(function() {
      this.timeout(60 * 1000);
      return rclnodejs.init();
    });

    after(function() {
      rclnodejs.shutdown();
    });

    it('Node.js subscription should receive msg from C++ publisher', (done) => {
      var node = rclnodejs.createNode('cpp_pub_js_sub');
      var rclString = rclnodejs.require('std_msgs').msg.String;
      var destroy = false;
      var cppTalker = childProcess.spawn('ros2', ['run', 'demo_nodes_cpp', 'talker']);
      var subscription = node.createSubscription(rclString, 'chatter', (msg) => {
        assert.ok(/Hello World:/.test(msg.data));             
        if (!destroy) {
          node.destroy();
          cppTalker.kill('SIGINT');
          destroy = true;          
        }
        done();
      });
      rclnodejs.spin(node);
    });

    it('Node.js subscription should receive msg from Python publisher', (done) => {
      var node = rclnodejs.createNode('cpp_pub_py_sub');
      var rclString = rclnodejs.require('std_msgs').msg.String;
      var destroy = false;
      var pyTalker = childProcess.spawn('ros2', ['run', 'demo_nodes_py', 'talker']);
      var subscription = node.createSubscription(rclString, 'chatter', (msg) => {
        assert.ok(/Hello World:/.test(msg.data));             
        if (!destroy) {
          node.destroy();
          pyTalker.kill('SIGINT');
          destroy = true;          
        }
        done();
      });
      rclnodejs.spin(node);
    });
  });
    
  describe('Node.js publisher', function() {
    before(function() {
      this.timeout(60 * 1000);
      return rclnodejs.init();
    });

    after(function() {
      rclnodejs.shutdown();
    });

    it('Cpp subscription should receive msg from Node.js publisher', (done) => {
      var node = rclnodejs.createNode('js_pub_cpp_sub');
      var rclString = rclnodejs.require('std_msgs').msg.String;
      var destroy = false;

      let text = 'Greeting from Node.js publisher';
      var cppListener = childProcess.spawn('ros2', ['run', 'demo_nodes_cpp', 'listener']);
      var publisher = node.createPublisher(rclString, 'chatter');
      var msg = new rclString();
      msg.data = text;
      var timer = setInterval(() => {
        publisher.publish(msg);
      }, 100);

      cppListener.stdout.on('data', (data) => {
        if (!destroy) {
          clearInterval(timer);
          assert.ok(new RegExp(text).test(data.toString()));
          done();
          node.destroy();
          cppListener.kill('SIGINT');
          destroy = true;
        }
      });
      rclnodejs.spin(node);
    });

    it('Python subscription should receive msg from Node.js publisher', function(done) {
      var node = rclnodejs.createNode('js_pub_py_sub');
      var rclString = rclnodejs.require('std_msgs').msg.String;
      var destroy = false;

      let text = 'Greeting from Node.js publisher';
      var pyListener = childProcess.spawn('python3', [`${__dirname}/py/listener.py`]);
      var publisher = node.createPublisher(rclString, 'chatter_py');
      var msg = new rclString();
      msg.data = text;

      pyListener.stdout.on('data', (data) => {
        if (!destroy) {
          clearInterval(timer);
          assert.ok(new RegExp(text).test(data.toString()));
          done();
          node.destroy();
          pyListener.kill('SIGINT');
          destroy = true;
        }
      });
      var timer = setInterval(() => {
        publisher.publish(msg);
      }, 100);
      rclnodejs.spin(node);
    });
  });
});
