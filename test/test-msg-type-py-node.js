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

/* eslint-disable camelcase */

const assert = require('assert');
const childProcess = require('child_process');
const deepEqual = require('deep-equal');
const rclnodejs = require('../index.js');
const utils = require('./utils.js');

describe('Rclnodejs - Python message type testing', function() {
  this.timeout(60 * 1000);

  before(function() {
    return rclnodejs.init();
  });

  after(function() {
    rclnodejs.shutdown();
  });

  describe('Primitive msg types', function(done) {
    this.timeout(60 * 1000);

    it('Bool', function(done) {
      var node = rclnodejs.createNode('bool_js_publisher');
      const Bool = 'std_msgs/msg/Bool';
      const msg = true;
      var destroy = false;

      var pySubscription = utils.launchPythonProcess([
        `${__dirname}/py/subscription_msg.py`,
        'Bool',
      ]);
      var publisher = node.createPublisher(Bool, 'Bool_js_py_channel');
      var subscription = node.createSubscription(
        Bool,
        'Bool_js_py_back_channel',
        backMsg => {
          assert.deepStrictEqual(msg, backMsg.data);
          timer.cancel();
          node.destroy();
          pySubscription.kill('SIGINT');
          done();
        }
      );
      var timer = node.createTimer(100, () => {
        publisher.publish(msg);
      });
      rclnodejs.spin(node);
    });

    it('Byte', function(done) {
      var node = rclnodejs.createNode('byte_js_publisher');
      const Byte = 'std_msgs/msg/Byte';
      const msg = 'A';
      var destroy = false;

      var pySubscription = utils.launchPythonProcess([
        `${__dirname}/py/subscription_msg.py`,
        'Byte',
      ]);
      var publisher = node.createPublisher(Byte, 'Byte_js_py_channel');
      var subscription = node.createSubscription(
        Byte,
        'Byte_js_py_back_channel',
        backMsg => {
          assert.deepStrictEqual(msg.charCodeAt(0), backMsg.data);
          timer.cancel();
          node.destroy();
          pySubscription.kill('SIGINT');
          done();
        }
      );
      var timer = node.createTimer(100, () => {
        publisher.publish(msg);
      });
      rclnodejs.spin(node);
    });

    it('Char', function(done) {
      var node = rclnodejs.createNode('char_js_publisher');
      const Char = 'std_msgs/msg/Char';
      const msg = 0x61;
      var destroy = false;

      var pySubscription = utils.launchPythonProcess([
        `${__dirname}/py/subscription_msg.py`,
        'Char',
      ]);
      var publisher = node.createPublisher(Char, 'Char_js_py_channel');
      var subscription = node.createSubscription(
        Char,
        'Char_js_py_back_channel',
        backMsg => {
          assert.deepStrictEqual(msg, backMsg.data);
          timer.cancel();
          node.destroy();
          pySubscription.kill('SIGINT');
          done();
        }
      );
      var timer = node.createTimer(100, () => {
        publisher.publish(msg);
      });
      rclnodejs.spin(node);
    });

    it('String', function(done) {
      var node = rclnodejs.createNode('string_js_publisher');
      const String = 'std_msgs/msg/String';
      const msg = 'Hello World';
      var destroy = false;

      var pySubscription = utils.launchPythonProcess([
        `${__dirname}/py/subscription_msg.py`,
        'String',
      ]);
      var publisher = node.createPublisher(String, 'String_js_py_channel');
      var subscription = node.createSubscription(
        String,
        'String_js_py_back_channel',
        backMsg => {
          assert.deepStrictEqual(msg, backMsg.data);
          timer.cancel();
          node.destroy();
          pySubscription.kill('SIGINT');
          done();
        }
      );
      var timer = node.createTimer(100, () => {
        publisher.publish(msg);
      });
      rclnodejs.spin(node);
    });

    it('Int8', function(done) {
      var node = rclnodejs.createNode('int8_js_publisher');
      const Int8 = 'std_msgs/msg/Int8';
      const msg = 0x7f;
      var destroy = false;

      var pySubscription = utils.launchPythonProcess([
        `${__dirname}/py/subscription_msg.py`,
        'Int8',
      ]);
      var publisher = node.createPublisher(Int8, 'Int8_js_py_channel');
      var subscription = node.createSubscription(
        Int8,
        'Int8_js_py_back_channel',
        backMsg => {
          assert.deepStrictEqual(msg, backMsg.data);
          timer.cancel();
          node.destroy();
          pySubscription.kill('SIGINT');
          done();
        }
      );
      var timer = node.createTimer(100, () => {
        publisher.publish(msg);
      });
      rclnodejs.spin(node);
    });

    it('UInt8', function(done) {
      var node = rclnodejs.createNode('uint8_js_publisher');
      const UInt8 = 'std_msgs/msg/UInt8';
      const msg = 0xff;
      var destroy = false;

      var pySubscription = utils.launchPythonProcess([
        `${__dirname}/py/subscription_msg.py`,
        'UInt8',
      ]);
      var publisher = node.createPublisher(UInt8, 'UInt8_js_py_channel');
      var subscription = node.createSubscription(
        UInt8,
        'UInt8_js_py_back_channel',
        backMsg => {
          assert.deepStrictEqual(msg, backMsg.data);
          timer.cancel();
          node.destroy();
          pySubscription.kill('SIGINT');
          done();
        }
      );
      var timer = node.createTimer(100, () => {
        publisher.publish(msg);
      });
      rclnodejs.spin(node);
    });

    it('Int16', function(done) {
      var node = rclnodejs.createNode('int16_js_publisher');
      const Int16 = 'std_msgs/msg/Int16';
      const msg = 0x7fff;
      var destroy = false;

      var pySubscription = utils.launchPythonProcess([
        `${__dirname}/py/subscription_msg.py`,
        'Int16',
      ]);
      var publisher = node.createPublisher(Int16, 'Int16_js_py_channel');
      var subscription = node.createSubscription(
        Int16,
        'Int16_js_py_back_channel',
        backMsg => {
          assert.deepStrictEqual(msg, backMsg.data);
          timer.cancel();
          node.destroy();
          pySubscription.kill('SIGINT');
          done();
        }
      );
      var timer = node.createTimer(100, () => {
        publisher.publish(msg);
      });
      rclnodejs.spin(node);
    });

    it('UInt16', function(done) {
      var node = rclnodejs.createNode('uint16_js_publisher');
      const UInt16 = 'std_msgs/msg/UInt16';
      const msg = 0xffff;
      var destroy = false;

      var pySubscription = utils.launchPythonProcess([
        `${__dirname}/py/subscription_msg.py`,
        'UInt16',
      ]);
      var publisher = node.createPublisher(UInt16, 'UInt16_js_py_channel');
      var subscription = node.createSubscription(
        UInt16,
        'UInt16_js_py_back_channel',
        backMsg => {
          assert.deepStrictEqual(msg, backMsg.data);
          timer.cancel();
          node.destroy();
          pySubscription.kill('SIGINT');
          done();
        }
      );
      var timer = node.createTimer(100, () => {
        publisher.publish(msg);
      });
      rclnodejs.spin(node);
    });

    it('Int32', function(done) {
      var node = rclnodejs.createNode('int32_js_publisher');
      const Int32 = 'std_msgs/msg/Int32';
      const msg = 0x7fffffff;
      var destroy = false;

      var pySubscription = utils.launchPythonProcess([
        `${__dirname}/py/subscription_msg.py`,
        'Int32',
      ]);
      var publisher = node.createPublisher(Int32, 'Int32_js_py_channel');
      var subscription = node.createSubscription(
        Int32,
        'Int32_js_py_back_channel',
        backMsg => {
          assert.deepStrictEqual(msg, backMsg.data);
          timer.cancel();
          node.destroy();
          pySubscription.kill('SIGINT');
          done();
        }
      );
      var timer = node.createTimer(100, () => {
        publisher.publish(msg);
      });
      rclnodejs.spin(node);
    });

    it('UInt32', function(done) {
      var node = rclnodejs.createNode('uint32_js_publisher');
      const UInt32 = 'std_msgs/msg/UInt32';
      const msg = 0xffffffff;
      var destroy = false;

      var pySubscription = utils.launchPythonProcess([
        `${__dirname}/py/subscription_msg.py`,
        'UInt32',
      ]);
      var publisher = node.createPublisher(UInt32, 'UInt32_js_py_channel');
      var subscription = node.createSubscription(
        UInt32,
        'UInt32_js_py_back_channel',
        backMsg => {
          assert.deepStrictEqual(msg, backMsg.data);
          timer.cancel();
          node.destroy();
          pySubscription.kill('SIGINT');
          done();
        }
      );
      var timer = node.createTimer(100, () => {
        publisher.publish(msg);
      });
      rclnodejs.spin(node);
    });

    it('Int64', function(done) {
      var node = rclnodejs.createNode('int64_js_publisher');
      const Int64 = 'std_msgs/msg/Int64';
      const msg = Number.MAX_SAFE_INTEGER;
      var destroy = false;

      var pySubscription = utils.launchPythonProcess([
        `${__dirname}/py/subscription_msg.py`,
        'Int64',
      ]);
      var publisher = node.createPublisher(Int64, 'Int64_js_py_channel');
      var subscription = node.createSubscription(
        Int64,
        'Int64_js_py_back_channel',
        backMsg => {
          assert.deepStrictEqual(msg, backMsg.data);
          timer.cancel();
          node.destroy();
          pySubscription.kill('SIGINT');
          done();
        }
      );
      var timer = node.createTimer(100, () => {
        publisher.publish(msg);
      });
      rclnodejs.spin(node);
    });

    it('UInt64', function(done) {
      var node = rclnodejs.createNode('uint64_js_publisher');
      const UInt64 = 'std_msgs/msg/UInt64';
      const msg = Number.MAX_SAFE_INTEGER;
      var destroy = false;

      var pySubscription = utils.launchPythonProcess([
        `${__dirname}/py/subscription_msg.py`,
        'UInt64',
      ]);
      var publisher = node.createPublisher(UInt64, 'UInt64_js_py_channel');
      var subscription = node.createSubscription(
        UInt64,
        'UInt64_js_py_back_channel',
        backMsg => {
          assert.deepStrictEqual(msg, backMsg.data);
          timer.cancel();
          node.destroy();
          pySubscription.kill('SIGINT');
          done();
        }
      );
      var timer = node.createTimer(100, () => {
        publisher.publish(msg);
      });
      rclnodejs.spin(node);
    });

    it('Float32', function(done) {
      var node = rclnodejs.createNode('float32_js_publisher');
      const Float32 = 'std_msgs/msg/Float32';
      const msg = 3.14;
      var destroy = false;

      var pySubscription = utils.launchPythonProcess([
        `${__dirname}/py/subscription_msg.py`,
        'Float32',
      ]);
      var publisher = node.createPublisher(Float32, 'Float32_js_py_channel');
      var subscription = node.createSubscription(
        Float32,
        'Float32_js_py_back_channel',
        backMsg => {
          assert.ok(Math.abs(msg - backMsg.data) < 0.01);
          timer.cancel();
          node.destroy();
          pySubscription.kill('SIGINT');
          done();
        }
      );
      var timer = node.createTimer(100, () => {
        publisher.publish(msg);
      });
      rclnodejs.spin(node);
    });

    it('Float64', function(done) {
      var node = rclnodejs.createNode('float64_js_publisher');
      const Float64 = 'std_msgs/msg/Float64';
      const msg = 3.1415926;
      var destroy = false;

      var pySubscription = utils.launchPythonProcess([
        `${__dirname}/py/subscription_msg.py`,
        'Float64',
      ]);
      var publisher = node.createPublisher(Float64, 'Float64_js_py_channel');
      var subscription = node.createSubscription(
        Float64,
        'Float64_js_py_back_channel',
        backMsg => {
          assert.ok(Math.abs(msg - backMsg.data) < 0.0000001);
          timer.cancel();
          node.destroy();
          pySubscription.kill('SIGINT');
          done();
        }
      );
      var timer = node.createTimer(100, () => {
        publisher.publish(msg);
      });
      rclnodejs.spin(node);
    });
  });

  describe('Compound msg types', function() {
    this.timeout(60 * 1000);

    it('Array', function(done) {
      var node = rclnodejs.createNode('multiarray_js_publisher');
      const ByteMultiArray = 'std_msgs/msg/ByteMultiArray';

      const msg = {
        layout: {
          dim: [
            {
              label: 'length',
              size: 1,
              stride: 3,
            },
          ],
          data_offset: 0,
        },
        data: [65, 66, 67],
      };

      var destroy = false;
      var pySubscription = utils.launchPythonProcess([
        `${__dirname}/py/subscription_msg.py`,
        'Array',
      ]);
      var publisher = node.createPublisher(
        ByteMultiArray,
        'Array_js_py_channel'
      );
      var subscription = node.createSubscription(
        ByteMultiArray,
        'Array_js_py_back_channel',
        backMsg => {
          assert.ok(deepEqual(msg, backMsg));
          timer.cancel();
          node.destroy();
          pySubscription.kill('SIGINT');
          done();
        }
      );
      var timer = node.createTimer(100, () => {
        publisher.publish(msg);
      });
      rclnodejs.spin(node);
    });

    it('ColorRGBA', function(done) {
      var node = rclnodejs.createNode('colorrgba_js_publisher');
      const ColorRGBA = 'std_msgs/msg/ColorRGBA';
      const msg = {
        a: 0.5,
        r: 127,
        g: 255,
        b: 255,
      };

      var destroy = false;
      var pySubscription = utils.launchPythonProcess([
        `${__dirname}/py/subscription_msg.py`,
        'ColorRGBA',
      ]);
      var publisher = node.createPublisher(
        ColorRGBA,
        'ColorRGBA_js_py_channel'
      );
      var subscription = node.createSubscription(
        ColorRGBA,
        'ColorRGBA_js_py_back_channel',
        backMsg => {
          assert.ok(deepEqual(msg, backMsg));
          timer.cancel();
          node.destroy();
          pySubscription.kill('SIGINT');
          done();
        }
      );
      var timer = node.createTimer(100, () => {
        publisher.publish(msg);
      });
      rclnodejs.spin(node);
    });

    it('Header', function(done) {
      var node = rclnodejs.createNode('header_js_publisher');
      const Header = 'std_msgs/msg/Header';

      const msg = {
        stamp: {
          sec: 123456,
          nanosec: 789,
        },
        frame_id: 'main frame',
      };

      var destroy = false;
      var pySubscription = utils.launchPythonProcess([
        `${__dirname}/py/subscription_msg.py`,
        'Header',
      ]);
      var publisher = node.createPublisher(Header, 'Header_js_py_channel');
      var subscription = node.createSubscription(
        Header,
        'Header_js_py_back_channel',
        backMsg => {
          assert.deepStrictEqual(msg, backMsg);
          timer.cancel();
          node.destroy();
          pySubscription.kill('SIGINT');
          done();
        }
      );
      var timer = node.createTimer(100, () => {
        publisher.publish(msg);
      });
      rclnodejs.spin(node);
    });

    it('Complex object', function(done) {
      var node = rclnodejs.createNode('jointstate_js_publisher');
      const JointState = 'sensor_msgs/msg/JointState';

      const msg = {
        header: {
          stamp: {
            sec: 123456,
            nanosec: 789,
          },
          frame_id: 'main frame',
        },
        name: ['Tom', 'Jerry'],
        position: [1, 2],
        velocity: [2, 3],
        effort: [4, 5, 6],
      };

      var destroy = false;
      var pySubscription = utils.launchPythonProcess([
        `${__dirname}/py/subscription_msg.py`,
        'JointState',
      ]);
      var publisher = node.createPublisher(
        JointState,
        'JointState_js_py_channel'
      );
      var subscription = node.createSubscription(
        JointState,
        'JointState_js_py_back_channel',
        backMsg => {
          assert.ok(deepEqual(msg, backMsg));
          timer.cancel();
          node.destroy();
          pySubscription.kill('SIGINT');
          done();
        }
      );
      var timer = node.createTimer(100, () => {
        publisher.publish(msg);
      });
      rclnodejs.spin(node);
    });
  });
});
