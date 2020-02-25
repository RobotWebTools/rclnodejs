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
const path = require('path');
const childProcess = require('child_process');
const deepEqual = require('deep-equal');
const rclnodejs = require('../index.js');
const utils = require('./utils.js');

describe('Rclnodejs - Cpp message type testing', function() {
  var cppSubscriptionPath = path.join(__dirname, 'cpp', 'subscription_msg');
  var cppSubscription;

  this.timeout(60 * 1000);

  before(function() {
    cppSubscription = childProcess.spawn(cppSubscriptionPath);
    return rclnodejs.init();
  });

  after(function() {
    rclnodejs.shutdown();
    cppSubscription.kill('SIGINT');
  });

  describe('Primitive message types', function() {
    it('Bool', function(done) {
      var node = rclnodejs.createNode('bool_js_publisher');
      const Bool = 'std_msgs/msg/Bool';
      const msg = true;

      var publisher = node.createPublisher(Bool, 'Bool_js_cpp_channel');
      var subscription = node.createSubscription(
        Bool,
        'back_Bool_js_cpp_channel',
        backMsg => {
          assert.deepStrictEqual(backMsg.data, msg);

          timer.cancel();
          node.destroy();
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
      const msg = 0x41;

      var publisher = node.createPublisher(Byte, 'Byte_js_cpp_channel');
      var subscription = node.createSubscription(
        Byte,
        'back_Byte_js_cpp_channel',
        backMsg => {
          assert.deepStrictEqual(backMsg.data, msg);

          timer.cancel();
          node.destroy();
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

      var publisher = node.createPublisher(Char, 'Char_js_cpp_channel');
      var subscription = node.createSubscription(
        Char,
        'back_Char_js_cpp_channel',
        backMsg => {
          assert.deepStrictEqual(backMsg.data, msg);

          timer.cancel();
          node.destroy();
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
      const RclString = 'std_msgs/msg/String';
      const msg = 'Hello World';

      var publisher = node.createPublisher(RclString, 'String_js_cpp_channel');
      var subscription = node.createSubscription(
        RclString,
        'back_String_js_cpp_channel',
        backMsg => {
          assert.deepStrictEqual(backMsg.data, msg);

          timer.cancel();
          node.destroy();
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

      var publisher = node.createPublisher(Int8, 'Int8_js_cpp_channel');
      var subscription = node.createSubscription(
        Int8,
        'back_Int8_js_cpp_channel',
        backMsg => {
          assert.deepStrictEqual(backMsg.data, msg);

          timer.cancel();
          node.destroy();
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

      var publisher = node.createPublisher(UInt8, 'UInt8_js_cpp_channel');
      var subscription = node.createSubscription(
        UInt8,
        'back_UInt8_js_cpp_channel',
        backMsg => {
          assert.deepStrictEqual(backMsg.data, msg);

          timer.cancel();
          node.destroy();
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

      var publisher = node.createPublisher(Int16, 'Int16_js_cpp_channel');
      var subscription = node.createSubscription(
        Int16,
        'back_Int16_js_cpp_channel',
        backMsg => {
          assert.deepStrictEqual(backMsg.data, msg);

          timer.cancel();
          node.destroy();
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

      var publisher = node.createPublisher(UInt16, 'UInt16_js_cpp_channel');
      var subscription = node.createSubscription(
        UInt16,
        'back_UInt16_js_cpp_channel',
        backMsg => {
          assert.deepStrictEqual(backMsg.data, msg);

          timer.cancel();
          node.destroy();
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

      var publisher = node.createPublisher(Int32, 'Int32_js_cpp_channel');
      var subscription = node.createSubscription(
        Int32,
        'back_Int32_js_cpp_channel',
        backMsg => {
          assert.deepStrictEqual(backMsg.data, msg);

          timer.cancel();
          node.destroy();
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

      var publisher = node.createPublisher(UInt32, 'UInt32_js_cpp_channel');
      var subscription = node.createSubscription(
        UInt32,
        'back_UInt32_js_cpp_channel',
        backMsg => {
          assert.deepStrictEqual(backMsg.data, msg);

          timer.cancel();
          node.destroy();
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

      var publisher = node.createPublisher(Int64, 'Int64_js_cpp_channel');
      var subscription = node.createSubscription(
        Int64,
        'back_Int64_js_cpp_channel',
        backMsg => {
          assert.deepStrictEqual(backMsg.data, msg);

          timer.cancel();
          node.destroy();
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

      var publisher = node.createPublisher(UInt64, 'UInt64_js_cpp_channel');
      var subscription = node.createSubscription(
        UInt64,
        'back_UInt64_js_cpp_channel',
        backMsg => {
          assert.deepStrictEqual(backMsg.data, msg);

          timer.cancel();
          node.destroy();
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

      var publisher = node.createPublisher(Float32, 'Float32_js_cpp_channel');
      var subscription = node.createSubscription(
        Float32,
        'back_Float32_js_cpp_channel',
        backMsg => {
          assert.ok(Math.abs(msg - backMsg.data) < 0.01);

          timer.cancel();
          node.destroy();
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

      var publisher = node.createPublisher(Float64, 'Float64_js_cpp_channel');
      var subscription = node.createSubscription(
        Float64,
        'back_Float64_js_cpp_channel',
        backMsg => {
          assert.ok(Math.abs(msg - backMsg.data) < 0.0000001);

          timer.cancel();
          node.destroy();
          done();
        }
      );
      var timer = node.createTimer(100, () => {
        publisher.publish(msg);
      });
      rclnodejs.spin(node);
    });
  });

  describe('Compound message types', function() {
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

      var publisher = node.createPublisher(
        ColorRGBA,
        'ColorRGBA_js_cpp_channel'
      );
      var subscription = node.createSubscription(
        ColorRGBA,
        'back_ColorRGBA_js_cpp_channel',
        backMsg => {
          assert.ok(deepEqual(msg, backMsg));

          timer.cancel();
          node.destroy();
          done();
        }
      );

      var timer = node.createTimer(100, () => {
        publisher.publish(msg);
      });
      rclnodejs.spin(node);
    });

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

      var publisher = node.createPublisher(
        ByteMultiArray,
        'Array_js_cpp_channel'
      );
      var subscription = node.createSubscription(
        ByteMultiArray,
        'back_ByteMultiArray_js_cpp_channel',
        backMsg => {
          assert.ok(deepEqual(msg, backMsg));

          timer.cancel();
          node.destroy();
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

      var publisher = node.createPublisher(Header, 'Header_js_cpp_channel');
      var subscription = node.createSubscription(
        Header,
        'back_Header_js_cpp_channel',
        backMsg => {
          assert.ok(deepEqual(msg, backMsg));

          timer.cancel();
          node.destroy();
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
      var publisher = node.createPublisher(
        JointState,
        'JointState_js_cpp_channel'
      );
      var subscription = node.createSubscription(
        JointState,
        'back_JointState_js_cpp_channel',
        backMsg => {
          assert.ok(deepEqual(msg, backMsg));

          timer.cancel();
          node.destroy();
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
