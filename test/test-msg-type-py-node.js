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

  describe('Python publisher - rclnodejs subscription: primitive msg types', function() {
    it('Bool', function(done) {
      var node = rclnodejs.createNode('bool_js_subscription');
      const Bool = 'std_msgs/msg/Bool';
      var destroy = false;
      var publisher = utils.launchPythonProcess([`${__dirname}/py/publisher_msg.py`, 'Bool']);
      var subscription = node.createSubscription(Bool, 'Bool_py_js_channel', (msg) => {
        assert.ok(msg.data);
        if (!destroy) {
          node.destroy();
          publisher.kill('SIGINT');
          destroy = true;
        }
        done();
      });
      rclnodejs.spin(node);
    });

    it('Byte', function(done) {
      var node = rclnodejs.createNode('byte_js_subscription');
      const Byte = 'std_msgs/msg/Byte';
      var destroy = false;
      var publisher = utils.launchPythonProcess([`${__dirname}/py/publisher_msg.py`, 'Byte']);
      var subscription = node.createSubscription(Byte, 'Byte_py_js_channel', (msg) => {
        assert.deepStrictEqual(msg.data, 255);
        if (!destroy) {
          node.destroy();
          publisher.kill('SIGINT');
          destroy = true;
        }
        done();
      });
      rclnodejs.spin(node);
    });

    it('Char', function(done) {
      var node = rclnodejs.createNode('char_js_subscription');
      const Char = 'std_msgs/msg/Char';
      var destroy = false;
      var publisher = utils.launchPythonProcess([`${__dirname}/py/publisher_msg.py`, 'Char']);
      var subscription = node.createSubscription(Char, 'Char_py_js_channel', (msg) => {
        assert.deepStrictEqual(msg.data, 65);
        if (!destroy) {
          node.destroy();
          publisher.kill('SIGINT');
          destroy = true;
        }
        done();
      });
      rclnodejs.spin(node);
    });

    it('String', function(done) {
      var node = rclnodejs.createNode('string_js_subscription');
      const String = 'std_msgs/msg/String';
      var destroy = false;
      var publisher = utils.launchPythonProcess([`${__dirname}/py/publisher_msg.py`, 'String']);
      var subscription = node.createSubscription(String, 'String_py_js_channel', (msg) => {
        assert.deepStrictEqual(msg.data, 'Hello World');
        if (!destroy) {
          node.destroy();
          publisher.kill('SIGINT');
          destroy = true;
        }
        done();
      });
      rclnodejs.spin(node);
    });

    it('Int8', function(done) {
      var node = rclnodejs.createNode('int8_js_subscription');
      const Int8 = 'std_msgs/msg/Int8';
      var destroy = false;
      var publisher = utils.launchPythonProcess([`${__dirname}/py/publisher_msg.py`, 'Int8']);
      var subscription = node.createSubscription(Int8, 'Int8_py_js_channel', (msg) => {
        assert.deepStrictEqual(msg.data, 127);
        if (!destroy) {
          node.destroy();
          publisher.kill('SIGINT');
          destroy = true;
        }
        done();
      });
      rclnodejs.spin(node);
    });

    it('UInt8', function(done) {
      var node = rclnodejs.createNode('uint8_js_subscription');
      const UInt8 = 'std_msgs/msg/UInt8';
      var destroy = false;
      var publisher = utils.launchPythonProcess([`${__dirname}/py/publisher_msg.py`, 'UInt8']);
      var subscription = node.createSubscription(UInt8, 'UInt8_py_js_channel', (msg) => {
        assert.deepStrictEqual(msg.data, 255);
        if (!destroy) {
          node.destroy();
          publisher.kill('SIGINT');
          destroy = true;
        }
        done();
      });
      rclnodejs.spin(node);
    });

    it('Int16', function(done) {
      var node = rclnodejs.createNode('int16_js_subscription');
      const Int16 = 'std_msgs/msg/Int16';
      var destroy = false;
      var publisher = utils.launchPythonProcess([`${__dirname}/py/publisher_msg.py`, 'Int16']);
      var subscription = node.createSubscription(Int16, 'Int16_py_js_channel', (msg) => {
        assert.deepStrictEqual(msg.data, 0x7fff);
        if (!destroy) {
          node.destroy();
          publisher.kill('SIGINT');
          destroy = true;
        }
        done();
      });
      rclnodejs.spin(node);
    });

    it('UInt16', function(done) {
      var node = rclnodejs.createNode('uint16_js_subscription');
      const UInt16 = 'std_msgs/msg/UInt16';
      var destroy = false;
      var publisher = utils.launchPythonProcess([`${__dirname}/py/publisher_msg.py`, 'UInt16']);
      var subscription = node.createSubscription(UInt16, 'UInt16_py_js_channel', (msg) => {
        assert.deepStrictEqual(msg.data, 0xffff);
        if (!destroy) {
          node.destroy();
          publisher.kill('SIGINT');
          destroy = true;
        }
        done();
      });
      rclnodejs.spin(node);
    });

    it('Int32', function(done) {
      var node = rclnodejs.createNode('int32_js_subscription');
      const Int32 = 'std_msgs/msg/Int32';
      var destroy = false;
      var publisher = utils.launchPythonProcess([`${__dirname}/py/publisher_msg.py`, 'Int32']);
      var subscription = node.createSubscription(Int32, 'Int32_py_js_channel', (msg) => {
        assert.deepStrictEqual(msg.data, 0x7fffffff);
        if (!destroy) {
          node.destroy();
          publisher.kill('SIGINT');
          destroy = true;
        }
        done();
      });
      rclnodejs.spin(node);
    });

    it('UInt32', function(done) {
      var node = rclnodejs.createNode('uint32_js_subscription');
      const UInt32 = 'std_msgs/msg/UInt32';
      var destroy = false;
      var publisher = utils.launchPythonProcess([`${__dirname}/py/publisher_msg.py`, 'UInt32']);
      var subscription = node.createSubscription(UInt32, 'UInt32_py_js_channel', (msg) => {
        assert.deepStrictEqual(msg.data, 0xffffffff);
        if (!destroy) {
          node.destroy();
          publisher.kill('SIGINT');
          destroy = true;
        }
        done();
      });
      rclnodejs.spin(node);
    });

    it('Int64', function(done) {
      var node = rclnodejs.createNode('int64_js_subscription');
      const Int64 = 'std_msgs/msg/Int64';
      var destroy = false;
      var publisher = utils.launchPythonProcess([`${__dirname}/py/publisher_msg.py`, 'Int64']);
      var subscription = node.createSubscription(Int64, 'Int64_py_js_channel', (msg) => {
        assert.deepStrictEqual(msg.data, Number.MAX_SAFE_INTEGER);
        if (!destroy) {
          node.destroy();
          publisher.kill('SIGINT');
          destroy = true;
        }
        done();
      });
      rclnodejs.spin(node);
    });

    it('UInt64', function(done) {
      var node = rclnodejs.createNode('uint64_js_subscription');
      const UInt64 = 'std_msgs/msg/UInt64';
      var destroy = false;
      var publisher = utils.launchPythonProcess([`${__dirname}/py/publisher_msg.py`, 'UInt64']);
      var subscription = node.createSubscription(UInt64, 'UInt64_py_js_channel', (msg) => {
        assert.deepStrictEqual(msg.data, Number.MAX_SAFE_INTEGER);
        if (!destroy) {
          node.destroy();
          publisher.kill('SIGINT');
          destroy = true;
        }
        done();
      });
      rclnodejs.spin(node);
    });

    it('Float32', function(done) {
      var node = rclnodejs.createNode('float32_js_subscription');
      const Float32 = 'std_msgs/msg/Float32';
      var destroy = false;
      var publisher = utils.launchPythonProcess([`${__dirname}/py/publisher_msg.py`, 'Float32']);
      var subscription = node.createSubscription(Float32, 'Float32_py_js_channel', (msg) => {
        assert.ok(Math.abs(msg.data - 3.14) < 0.000001);
        if (!destroy) {
          node.destroy();
          publisher.kill('SIGINT');
          destroy = true;
        }
        done();
      });
      rclnodejs.spin(node);
    });

    it('Float64', function(done) {
      var node = rclnodejs.createNode('float64_js_subscription');
      const Float64 = 'std_msgs/msg/Float64';
      var destroy = false;
      var publisher = utils.launchPythonProcess([`${__dirname}/py/publisher_msg.py`, 'Float64']);
      var subscription = node.createSubscription(Float64, 'Float64_py_js_channel', (msg) => {
        assert.ok(Math.abs(msg.data - 3.14) < 0.000001);
        if (!destroy) {
          node.destroy();
          publisher.kill('SIGINT');
          destroy = true;
        }
        done();
      });
      rclnodejs.spin(node);
    });
  });

  describe('Python publisher - rlcnodejs subscription: compound msg types', function() {
    this.timeout(60 * 1000);

    it('ColorRGBA', function(done) {
      var node = rclnodejs.createNode('colorrgba_js_subscription');
      const ColorRGBA = 'std_msgs/msg/ColorRGBA';
      var destroy = false;
      var publisher = utils.launchPythonProcess([`${__dirname}/py/publisher_msg.py`, 'ColorRGBA']);
      var subscription = node.createSubscription(ColorRGBA, 'ColorRGBA_py_js_channel', (msg) => {
        assert.ok(Math.abs(msg.a - 0.5) < 0.000001);
        assert.ok(Math.abs(msg.r - 127) < 0.000001);
        assert.ok(Math.abs(msg.g - 255) < 0.000001);
        assert.ok(Math.abs(msg.b - 255) < 0.000001);

        if (!destroy) {
          node.destroy();
          publisher.kill('SIGINT');
          destroy = true;
        }
        done();
      });
      rclnodejs.spin(node);
    });

    it('Array', function(done) {
      var node = rclnodejs.createNode('array_js_subscription');
      const ByteMultiArray = 'std_msgs/msg/ByteMultiArray';
      var destroy = false;
      var publisher = utils.launchPythonProcess([`${__dirname}/py/publisher_msg.py`, 'Array']);
      var subscription = node.createSubscription(ByteMultiArray, 'Array_py_js_channel', (msg) => {
        assert.deepStrictEqual(msg.data, Uint8Array.from([65, 66, 67]));

        if (!destroy) {
          node.destroy();
          publisher.kill('SIGINT');
          destroy = true;
        }
        done();
      });
      rclnodejs.spin(node);
    });

    it('Header', function(done) {
      var node = rclnodejs.createNode('header_js_publisher');
      const Header = 'std_msgs/msg/Header';
      var destroy = false;
      var publisher = utils.launchPythonProcess([`${__dirname}/py/publisher_msg.py`, 'Header']);
      var subscription = node.createSubscription(Header, 'Header_py_js_channel', (msg) => {
        assert.deepStrictEqual(msg.stamp.sec, 123456);
        assert.deepStrictEqual(msg.stamp.nanosec, 789);
        assert.deepStrictEqual(msg.frame_id, 'main frame');

        if (!destroy) {
          node.destroy();
          publisher.kill('SIGINT');
          destroy = true;
        }
        done();
      });
      rclnodejs.spin(node);
    });

    it('Complex object', function(done) {
      var node = rclnodejs.createNode('jointstate_js_publisher');
      const JointState = 'sensor_msgs/msg/JointState';
      var destroy = false;
      var publisher = utils.launchPythonProcess([`${__dirname}/py/publisher_msg.py`, 'JointState']);
      var subscription = node.createSubscription(JointState, 'JointState_py_js_channel', (msg) => {
        assert.deepStrictEqual(msg.header.stamp.sec, 123456);
        assert.deepStrictEqual(msg.header.stamp.nanosec, 789);
        assert.deepStrictEqual(msg.header.frame_id, 'main frame');
        assert.deepStrictEqual(msg.name, ['Tom', 'Jerry']);
        assert.deepStrictEqual(msg.position, Float64Array.from([1, 2]));
        assert.deepStrictEqual(msg.velocity, Float64Array.from([2, 3]));
        assert.deepStrictEqual(msg.effort, Float64Array.from([4, 5, 6]));

        if (!destroy) {
          node.destroy();
          publisher.kill('SIGINT');
          destroy = true;
        }
        done();
      });
      rclnodejs.spin(node);
    });
  });

  describe('Rclnodejs publisher - Python subscription: primitive msg types', function(done) {
    this.timeout(60 * 1000);

    it('Bool', function(done) {
      var node = rclnodejs.createNode('bool_js_publisher');
      const Bool = 'std_msgs/msg/Bool';
      const msg = true;
      var destroy = false;

      var subscription = utils.launchPythonProcess([`${__dirname}/py/subscription_msg.py`, 'Bool']);
      var publisher = node.createPublisher(Bool, 'Bool_js_py_channel');

      const expected = 'True';
      subscription.stdout.on('data', (data) => {
        if (!destroy) {
          assert.ok(new RegExp(expected).test(data.toString()));
          timer.cancel();
          node.destroy();
          subscription.kill('SIGINT');
          destroy = true;
          done();
        }
      });
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

      var subscription = utils.launchPythonProcess([`${__dirname}/py/subscription_msg.py`, 'Byte']);
      var publisher = node.createPublisher(Byte, 'Byte_js_py_channel');

      const expected = "b'A'";
      subscription.stdout.on('data', (data) => {
        if (!destroy) {
          assert.ok(new RegExp(expected).test(data.toString()));
          timer.cancel();
          node.destroy();
          subscription.kill('SIGINT');
          destroy = true;
          done();
        }
      });
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

      var subscription = utils.launchPythonProcess([`${__dirname}/py/subscription_msg.py`, 'Char']);
      var publisher = node.createPublisher(Char, 'Char_js_py_channel');

      const expected = 'a';
      subscription.stdout.on('data', (data) => {
        if (!destroy) {
          assert.ok(new RegExp(expected).test(data.toString()));
          timer.cancel();
          node.destroy();
          subscription.kill('SIGINT');
          destroy = true;
          done();
        }
      });
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

      var subscription = utils.launchPythonProcess([`${__dirname}/py/subscription_msg.py`, 'String']);
      var publisher = node.createPublisher(String, 'String_js_py_channel');

      const expected = 'Hello World';
      subscription.stdout.on('data', (data) => {
        if (!destroy) {
          assert.ok(new RegExp(expected).test(data.toString()));
          timer.cancel();
          node.destroy();
          subscription.kill('SIGINT');
          destroy = true;
          done();
        }
      });
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

      var subscription = utils.launchPythonProcess([`${__dirname}/py/subscription_msg.py`, 'Int8']);
      var publisher = node.createPublisher(Int8, 'Int8_js_py_channel');

      const expected = '127';
      subscription.stdout.on('data', (data) => {
        if (!destroy) {
          assert.ok(new RegExp(expected).test(data.toString()));
          timer.cancel();
          node.destroy();
          subscription.kill('SIGINT');
          destroy = true;
          done();
        }
      });
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

      var subscription = utils.launchPythonProcess([`${__dirname}/py/subscription_msg.py`, 'UInt8']);
      var publisher = node.createPublisher(UInt8, 'UInt8_js_py_channel');

      const expected = '255';
      subscription.stdout.on('data', (data) => {
        if (!destroy) {
          assert.ok(new RegExp(expected).test(data.toString()));
          timer.cancel();
          node.destroy();
          subscription.kill('SIGINT');
          destroy = true;
          done();
        }
      });
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

      var subscription = utils.launchPythonProcess([`${__dirname}/py/subscription_msg.py`, 'Int16']);
      var publisher = node.createPublisher(Int16, 'Int16_js_py_channel');

      const expected = '32767';
      subscription.stdout.on('data', (data) => {
        if (!destroy) {
          assert.ok(new RegExp(expected).test(data.toString()));
          timer.cancel();
          node.destroy();
          subscription.kill('SIGINT');
          destroy = true;
          done();
        }
      });
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

      var subscription = utils.launchPythonProcess([`${__dirname}/py/subscription_msg.py`, 'UInt16']);
      var publisher = node.createPublisher(UInt16, 'UInt16_js_py_channel');

      const expected = '65535';
      subscription.stdout.on('data', (data) => {
        if (!destroy) {
          assert.ok(new RegExp(expected).test(data.toString()));
          timer.cancel();
          node.destroy();
          subscription.kill('SIGINT');
          destroy = true;
          done();
        }
      });
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

      var subscription = utils.launchPythonProcess([`${__dirname}/py/subscription_msg.py`, 'Int32']);
      var publisher = node.createPublisher(Int32, 'Int32_js_py_channel');

      const expected = '2147483647';
      subscription.stdout.on('data', (data) => {
        if (!destroy) {
          assert.ok(new RegExp(expected).test(data.toString()));
          timer.cancel();
          node.destroy();
          subscription.kill('SIGINT');
          destroy = true;
          done();
        }
      });
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

      var subscription = utils.launchPythonProcess([`${__dirname}/py/subscription_msg.py`, 'UInt32']);
      var publisher = node.createPublisher(UInt32, 'UInt32_js_py_channel');

      const expected = '4294967295';
      subscription.stdout.on('data', (data) => {
        if (!destroy) {
          assert.ok(new RegExp(expected).test(data.toString()));
          timer.cancel();
          node.destroy();
          subscription.kill('SIGINT');
          destroy = true;
          done();
        }
      });
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

      var subscription = utils.launchPythonProcess([`${__dirname}/py/subscription_msg.py`, 'Int64']);
      var publisher = node.createPublisher(Int64, 'Int64_js_py_channel');

      const expected = '9007199254740991';
      subscription.stdout.on('data', (data) => {
        if (!destroy) {
          assert.ok(new RegExp(expected).test(data.toString()));
          timer.cancel();
          node.destroy();
          subscription.kill('SIGINT');
          destroy = true;
          done();
        }
      });
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

      var subscription = utils.launchPythonProcess([`${__dirname}/py/subscription_msg.py`, 'UInt64']);
      var publisher = node.createPublisher(UInt64, 'UInt64_js_py_channel');

      const expected = '9007199254740991';
      subscription.stdout.on('data', (data) => {
        if (!destroy) {
          assert.ok(new RegExp(expected).test(data.toString()));
          timer.cancel();
          node.destroy();
          subscription.kill('SIGINT');
          destroy = true;
          done();
        }
      });
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

      var subscription = utils.launchPythonProcess([`${__dirname}/py/subscription_msg.py`, 'Float32']);
      var publisher = node.createPublisher(Float32, 'Float32_js_py_channel');

      const expected = '3.14';
      subscription.stdout.on('data', (data) => {
        if (!destroy) {
          assert.ok(Math.abs(parseFloat(data.toString()) - expected) < 0.000001);
          timer.cancel();
          node.destroy();
          subscription.kill('SIGINT');
          destroy = true;
          done();
        }
      });
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

      var subscription = utils.launchPythonProcess([`${__dirname}/py/subscription_msg.py`, 'Float64']);
      var publisher = node.createPublisher(Float64, 'Float64_js_py_channel');

      const expected = '3.1415926';
      subscription.stdout.on('data', (data) => {
        if (!destroy) {
          assert.ok(new RegExp(expected).test(data.toString()));
          timer.cancel();
          node.destroy();
          subscription.kill('SIGINT');
          destroy = true;
          done();
        }
      });
      var timer = node.createTimer(100, () => {
        publisher.publish(msg);
      });
      rclnodejs.spin(node);
    });
  });

  describe('Rclnodejs publisher - Python subscription: compound msg types', function() {
    this.timeout(60 * 1000);

    it('Array', function(done) {
      var node = rclnodejs.createNode('multiarray_js_publisher');
      const ByteMultiArray = 'std_msgs/msg/ByteMultiArray';

      const byteArray = {
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
      var subscription = utils.launchPythonProcess([`${__dirname}/py/subscription_msg.py`, 'Array']);
      var publisher = node.createPublisher(ByteMultiArray, 'Array_js_py_channel');
      const expected = 'ABC';
      subscription.stdout.on('data', (data) => {
        if (!destroy) {
          assert.ok(new RegExp(expected).test(data.toString()));
          timer.cancel();
          node.destroy();
          subscription.kill('SIGINT');
          destroy = true;
          done();
        }
      });
      var timer = node.createTimer(100, () => {
        publisher.publish(byteArray);
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
      var subscription = utils.launchPythonProcess([`${__dirname}/py/subscription_msg.py`, 'ColorRGBA']);
      var publisher = node.createPublisher(ColorRGBA, 'ColorRGBA_js_py_channel');
      const expected = '(127.0,255.0,255.0,0.5)';
      subscription.stdout.on('data', (data) => {
        if (!destroy) {
          assert.ok(new RegExp(expected).test(data.toString()));
          timer.cancel();
          node.destroy();
          subscription.kill('SIGINT');
          destroy = true;
          done();
        }
      });
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
      var subscription = utils.launchPythonProcess([`${__dirname}/py/subscription_msg.py`, 'Header']);
      var publisher = node.createPublisher(Header, 'Header_js_py_channel');
      const expected = '(123456,789,main frame)';
      subscription.stdout.on('data', (data) => {
        if (!destroy) {
          assert.ok(new RegExp(expected).test(data.toString()));
          timer.cancel();
          node.destroy();
          subscription.kill('SIGINT');
          destroy = true;
          done();
        }
      });
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
      var subscription = utils.launchPythonProcess([`${__dirname}/py/subscription_msg.py`, 'JointState']);
      var publisher = node.createPublisher(JointState, 'JointState_js_py_channel');
      const expected = "(123456,789,main frame,['Tom', 'Jerry'],[1.0, 2.0],[2.0, 3.0],[4.0, 5.0, 6.0])";
      subscription.stdout.on('data', (data) => {
        if (!destroy) {
          assert.notDeepStrictEqual(data.toString().indexOf(expected), -1);
          timer.cancel();
          node.destroy();
          subscription.kill('SIGINT');
          destroy = true;
          done();
        }       
      });
      var timer = node.createTimer(100, () => {
        publisher.publish(msg);
      });
      rclnodejs.spin(node);      
    });
  });
});
