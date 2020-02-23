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

// function checkType(m, rclType, jsType, value, callback) {
//                   // rclnodejs, 'Bool', 'boolean', '"true"', callback
//   if ((typeof rclType !== 'string') || (typeof value !== 'string')) {
//     throw TypeError('Invalid type of parameters!');
//   }

//   var node = m.createNode(rclType + '_subscription');
//   var msgType = m.require('std_msgs').msg[rclType];
//   var publisher = childProcess.fork(`${__dirname}/publisher_msg.js`, [rclType, value]);
//   var subscription = node.createSubscription(msgType, rclType + '_type_channel', (msg) => {
//     publisher.send('quit');
//     callback(msg, jsType, value);
//   });
//   m.spin(node);
// }

describe('Rclnodejs message type testing', function() {
  this.timeout(60 * 1000);

  beforeEach(function() {
    return rclnodejs.init();
  });

  afterEach(function() {
    rclnodejs.shutdown();
  });

  describe('Primitive types', function() {
    it('Bool', function(done) {
      var node = rclnodejs.createNode('bool_subscription');
      const msgBool = 'std_msgs/msg/Bool';
      var publisher = childProcess.fork(`${__dirname}/publisher_msg.js`, [
        'Bool',
        '"true"',
      ]);
      var subscription = node.createSubscription(
        msgBool,
        'Bool_channel',
        msg => {
          publisher.kill('SIGINT');
          assert.deepStrictEqual(typeof msg.data, 'boolean');
          assert.ok(msg.data);
          done();
        }
      );
      rclnodejs.spin(node);
    });

    it('Char', function(done) {
      var node = rclnodejs.createNode('char_subscription');
      const msgChar = 'std_msgs/msg/Char';
      var publisher = childProcess.fork(`${__dirname}/publisher_msg.js`, [
        'Char',
        '"A"',
      ]);
      var subscription = node.createSubscription(
        msgChar,
        'Char_channel',
        msg => {
          publisher.kill('SIGINT');
          assert.deepStrictEqual(typeof msg.data, 'number');
          assert.deepStrictEqual(msg.data, 65);
          done();
        }
      );
      rclnodejs.spin(node);
    });

    it('Byte', function(done) {
      var node = rclnodejs.createNode('byte_subscription');
      const msgByte = 'std_msgs/msg/Byte';
      var publisher = childProcess.fork(`${__dirname}/publisher_msg.js`, [
        'Byte',
        '0xa',
      ]);
      var subscription = node.createSubscription(
        msgByte,
        'Byte_channel',
        msg => {
          publisher.kill('SIGINT');
          assert.deepStrictEqual(typeof msg.data, 'number');
          assert.deepStrictEqual(msg.data, 10);
          done();
        }
      );
      rclnodejs.spin(node);
    });

    it('String', function(done) {
      var node = rclnodejs.createNode('string_subscription');
      var msgString = 'std_msgs/msg/String';
      var publisher = childProcess.fork(`${__dirname}/publisher_msg.js`, [
        'String',
        '"RCL String"',
      ]);
      var subscription = node.createSubscription(
        msgString,
        'String_channel',
        msg => {
          publisher.kill('SIGINT');
          assert.deepStrictEqual(typeof msg.data, 'string');
          assert.deepStrictEqual(msg.data, 'RCL String');
          done();
        }
      );
      rclnodejs.spin(node);
    });

    // it('InitString', function(done) {
    //   var node = rclnodejs.createNode('init_string_subscription');
    //   var msgString = 'std_msgs/msg/String';
    //   var publisher = childProcess.fork(`${__dirname}/publisher_msg.js`, ['InitString', '"RCL String"']);
    //   var subscription = node.createSubscription(msgString, 'InitString_channel', (msg) => {
    //     publisher.send('quit');
    //     assert.deepStrictEqual(typeof msg.data, 'string');
    //     assert.deepStrictEqual(msg.data, 'RCL String');
    //     done();
    //   });
    //   rclnodejs.spin(node);
    // });

    it('Int8', function(done) {
      var node = rclnodejs.createNode('int8_subscription');
      const msgInt8 = 'std_msgs/msg/Int8';
      var publisher = childProcess.fork(`${__dirname}/publisher_msg.js`, [
        'Int8',
        '0x7f',
      ]);
      var subscription = node.createSubscription(
        msgInt8,
        'Int8_channel',
        msg => {
          publisher.kill('SIGINT');
          assert.deepStrictEqual(typeof msg.data, 'number');
          assert.deepStrictEqual(msg.data, 127);
          done();
        }
      );
      rclnodejs.spin(node);
    });

    it('UInt8', function(done) {
      var node = rclnodejs.createNode('uint8_subscription');
      const msgUInt8 = 'std_msgs/msg/UInt8';
      var publisher = childProcess.fork(`${__dirname}/publisher_msg.js`, [
        'UInt8',
        '0xff',
      ]);
      var subscription = node.createSubscription(
        msgUInt8,
        'UInt8_channel',
        msg => {
          publisher.kill('SIGINT');
          assert.deepStrictEqual(typeof msg.data, 'number');
          assert.deepStrictEqual(msg.data, 255);
          done();
        }
      );
      rclnodejs.spin(node);
    });

    it('Int16', function(done) {
      var node = rclnodejs.createNode('int16_subscription');
      const msgInt16 = 'std_msgs/msg/Int16';
      var publisher = childProcess.fork(`${__dirname}/publisher_msg.js`, [
        'Int16',
        '0x7fff',
      ]);
      var subscription = node.createSubscription(
        msgInt16,
        'Int16_channel',
        msg => {
          publisher.kill('SIGINT');
          assert.deepStrictEqual(typeof msg.data, 'number');
          assert.deepStrictEqual(msg.data, 0x7fff);
          done();
        }
      );
      rclnodejs.spin(node);
    });

    it('UInt16', function(done) {
      var node = rclnodejs.createNode('uint16_subscription');
      const msgUInt16 = 'std_msgs/msg/UInt16';
      var publisher = childProcess.fork(`${__dirname}/publisher_msg.js`, [
        'UInt16',
        '0xffff',
      ]);
      var subscription = node.createSubscription(
        msgUInt16,
        'UInt16_channel',
        msg => {
          publisher.kill('SIGINT');
          assert.deepStrictEqual(typeof msg.data, 'number');
          assert.deepStrictEqual(msg.data, 0xffff);
          done();
        }
      );
      rclnodejs.spin(node);
    });

    it('Int32', function(done) {
      var node = rclnodejs.createNode('int32_subscription');
      const msgInt32 = 'std_msgs/msg/Int32';
      var publisher = childProcess.fork(`${__dirname}/publisher_msg.js`, [
        'Int32',
        '0x7fffffff',
      ]);
      var subscription = node.createSubscription(
        msgInt32,
        'Int32_channel',
        msg => {
          publisher.kill('SIGINT');
          assert.deepStrictEqual(typeof msg.data, 'number');
          assert.deepStrictEqual(msg.data, 0x7fffffff);
          done();
        }
      );
      rclnodejs.spin(node);
    });

    it('UInt32', function(done) {
      var node = rclnodejs.createNode('uint32_subscription');
      const msgUInt32 = 'std_msgs/msg/UInt32';
      var publisher = childProcess.fork(`${__dirname}/publisher_msg.js`, [
        'UInt32',
        '0xffffffff',
      ]);
      var subscription = node.createSubscription(
        msgUInt32,
        'UInt32_channel',
        msg => {
          publisher.kill('SIGINT');
          assert.deepStrictEqual(typeof msg.data, 'number');
          assert.deepStrictEqual(msg.data, 0xffffffff);
          done();
        }
      );
      rclnodejs.spin(node);
    });

    it('Int64', function(done) {
      var node = rclnodejs.createNode('int64_subscription');
      const msgInt64 = 'std_msgs/msg/Int64';
      var publisher = childProcess.fork(`${__dirname}/publisher_msg.js`, [
        'Int64',
        '0x1fffffffffffff',
      ]);
      var subscription = node.createSubscription(
        msgInt64,
        'Int64_channel',
        msg => {
          publisher.kill('SIGINT');
          // assert.deepStrictEqual(typeof msg.data, 'string');
          assert.deepStrictEqual(
            parseInt(msg.data, 10),
            Number.MAX_SAFE_INTEGER
          );
          done();
        }
      );
      rclnodejs.spin(node);
    });

    it('UInt64', function(done) {
      var node = rclnodejs.createNode('uint64_subscription');
      const msgUInt64 = 'std_msgs/msg/UInt64';
      var publisher = childProcess.fork(`${__dirname}/publisher_msg.js`, [
        'UInt64',
        '0x1fffffffffffff',
      ]);
      var subscription = node.createSubscription(
        msgUInt64,
        'UInt64_channel',
        msg => {
          publisher.kill('SIGINT');
          // assert.deepStrictEqual(typeof msg.data, 'string');
          assert.deepStrictEqual(
            parseInt(msg.data, 10),
            Number.MAX_SAFE_INTEGER
          );
          done();
        }
      );
      rclnodejs.spin(node);
    });

    it('Float32', function(done) {
      var node = rclnodejs.createNode('float32_subscription');
      const msgFloat32 = 'std_msgs/msg/Float32';
      var publisher = childProcess.fork(`${__dirname}/publisher_msg.js`, [
        'Float32',
        '3.14',
      ]);
      var subscription = node.createSubscription(
        msgFloat32,
        'Float32_channel',
        msg => {
          publisher.kill('SIGINT');
          assert.deepStrictEqual(typeof msg.data, 'number');
          assert.ok(Math.abs(msg.data - 3.14) < 0.000001);
          done();
        }
      );
      rclnodejs.spin(node);
    });

    it('Float64', function(done) {
      var node = rclnodejs.createNode('float64_subscription');
      const msgFloat64 = 'std_msgs/msg/Float64';
      var publisher = childProcess.fork(`${__dirname}/publisher_msg.js`, [
        'Float64',
        '3.14',
      ]);
      var subscription = node.createSubscription(
        msgFloat64,
        'Float64_channel',
        msg => {
          publisher.kill('SIGINT');
          assert.deepStrictEqual(typeof msg.data, 'number');
          assert.ok(Math.abs(msg.data - 3.14) < 0.000001);
          done();
        }
      );
      rclnodejs.spin(node);
    });
  });

  describe('Compound types', function() {
    this.timeout(60 * 1000);
    it('ColorRGBA', function(done) {
      var node = rclnodejs.createNode('colorrgba_subscription');
      const msgColorRGBA = 'std_msgs/msg/ColorRGBA';
      var publisher = childProcess.fork(
        `${__dirname}/publisher_msg_colorrgba.js`
      );
      var subscription = node.createSubscription(
        msgColorRGBA,
        'ColorRGBA_channel',
        msg => {
          publisher.kill('SIGINT');
          assert.ok('r' in msg);
          // assert.deepStrictEqual(typeof msg.r, 'number');
          assert.deepStrictEqual(msg.r, 127);
          assert.ok('g' in msg);
          // assert.deepStrictEqual(typeof msg.g, 'number');
          assert.deepStrictEqual(msg.g, 255);
          assert.ok('b' in msg);
          // assert.deepStrictEqual(typeof msg.b, 'number');
          assert.deepStrictEqual(msg.b, 255);
          assert.ok('a' in msg);
          // assert.deepStrictEqual(typeof msg.a, 'number');
          assert.ok(Math.abs(msg.a - 0.5) < 0.000001);
          done();
        }
      );
      rclnodejs.spin(node);
    });

    // it('Array', function(done) {

    // });

    it('Object with Header', function(done) {
      var node = rclnodejs.createNode('header_subscription');
      const Header = 'std_msgs/msg/Header';
      var publisher = childProcess.fork(`${__dirname}/publisher_msg_header.js`);
      var subscription = node.createSubscription(
        Header,
        'Header_channel',
        header => {
          publisher.kill('SIGINT');

          assert.ok('stamp' in header);
          assert.ok('sec' in header.stamp);
          assert.deepStrictEqual(header.stamp.sec, 123456);
          assert.ok('nanosec' in header.stamp);
          assert.deepStrictEqual(header.stamp.nanosec, 789);
          assert.ok('frame_id' in header);
          assert.deepStrictEqual(header.frame_id, 'main frame');

          done();
        }
      );
      rclnodejs.spin(node);
    });

    it('Complex object', function(done) {
      var node = rclnodejs.createNode('jointstate_subscription');
      const JointState = 'sensor_msgs/msg/JointState';
      var publisher = childProcess.fork(
        `${__dirname}/publisher_msg_jointstate.js`
      );
      var subscription = node.createSubscription(
        JointState,
        'JointState_channel',
        state => {
          publisher.kill('SIGINT');

          assert.deepStrictEqual(state.name, ['Tom', 'Jerry']);
          assert.deepStrictEqual(state.position, Float64Array.from([1, 2]));
          assert.deepStrictEqual(state.velocity, Float64Array.from([2, 3]));
          assert.deepStrictEqual(state.effort, Float64Array.from([4, 5, 6]));

          done();
        }
      );
      rclnodejs.spin(node);
    });
  });
});
