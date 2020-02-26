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
const assertThrowsError = require('./utils.js').assertThrowsError;

describe('Rclnodejs message type data testing', function() {
  this.timeout(60 * 1000);

  before(function() {
    return rclnodejs.init();
  });

  after(function() {
    rclnodejs.shutdown();
  });

  it('Bool data checking', function() {
    var node = rclnodejs.createNode('bool');
    var msgBool = rclnodejs.require('std_msgs').msg.Bool;
    var msg = new msgBool();

    msg.data = true;
    assert.ok(msg.data);

    msg.data = false;
    assert.ok(!msg.data);

    msg.data = 1;
    assert.ok(msg.data);

    msg.data = 0;
    assert.ok(!msg.data);

    msg.data = 'ABc';
    assert.ok(msg.data);

    msg.data = '';
    assert.ok(!msg.data);
  });

  it('Char data checking', function() {
    var node = rclnodejs.createNode('char');
    var msgChar = rclnodejs.require('std_msgs').msg.Char;
    var msg = new msgChar();

    msg.data = 'A';
    assert.deepStrictEqual(typeof msg.data, 'number');
    assert.deepStrictEqual(msg.data, 65);

    assertThrowsError(
      () => {
        msg.data = -129;
      },
      [TypeError, RangeError],
      'out of bounds',
      'Char should be in [-128, 127]'
    );
    assertThrowsError(
      () => {
        msg.data = 128;
      },
      [TypeError, RangeError],
      'out of bounds',
      'Char should be in [-128, 127]'
    );
  });

  it('Byte data checking', function() {
    var node = rclnodejs.createNode('byte');
    var msgByte = rclnodejs.require('std_msgs').msg.Byte;
    var msg = new msgByte();

    msg.data = 0xa;
    assert.deepStrictEqual(typeof msg.data, 'number');
    assert.deepStrictEqual(msg.data, 10);

    assertThrowsError(
      () => {
        msg.data = -1;
      },
      [TypeError, RangeError],
      'out of bounds',
      'Byte should be in [0, 255]'
    );
    assertThrowsError(
      () => {
        msg.data = 256;
      },
      [TypeError, RangeError],
      'out of bounds',
      'Byte should be in [0, 255]'
    );
  });

  it('String data checking', function() {
    var node = rclnodejs.createNode('string');
    var msgString = rclnodejs.require('std_msgs').msg.String;
    var msg = new msgString();

    msg.data = 'rclnodejs';
    assert.deepStrictEqual(typeof msg.data, 'string');
    assert.deepStrictEqual(msg.data, 'rclnodejs');

    assertThrowsError(
      () => {
        msg.data = 1;
      },
      [TypeError, RangeError],
      'Number/String 64-bit value required',
      'String data is not a string'
    );
    assertThrowsError(
      () => {
        msg.data = 3.14;
      },
      [TypeError, RangeError],
      'Number/String 64-bit value required',
      'String data is not a string'
    );
  });

  it('Int8 data checking', function() {
    var node = rclnodejs.createNode('int8');
    var msgInt8 = rclnodejs.require('std_msgs').msg.Int8;
    var msg = new msgInt8();

    msg.data = 0x7f;
    assert.deepStrictEqual(typeof msg.data, 'number');
    assert.deepStrictEqual(msg.data, 127);

    assertThrowsError(
      () => {
        msg.data = -129;
      },
      [TypeError, RangeError],
      'out of bounds',
      'Int8 should be in [-128, 127]'
    );
    assertThrowsError(
      () => {
        msg.data = 128;
      },
      [TypeError, RangeError],
      'out of bounds',
      'Int8 should be in [-128, 127]'
    );
  });

  it('UInt8 data checking', function() {
    var node = rclnodejs.createNode('uint8');
    var msgUInt8 = rclnodejs.require('std_msgs').msg.UInt8;
    var msg = new msgUInt8();

    msg.data = 0xff;
    assert.deepStrictEqual(typeof msg.data, 'number');
    assert.deepStrictEqual(msg.data, 255);

    assertThrowsError(
      () => {
        msg.data = -1;
      },
      [TypeError, RangeError],
      'out of bounds',
      'UInt8 should be in [0, 256]'
    );
    assertThrowsError(
      () => {
        msg.data = 256;
      },
      [TypeError, RangeError],
      'out of bounds',
      'UInt8 should be in [0, 256]'
    );
  });

  it('Int16 data checking', function() {
    var node = rclnodejs.createNode('int16');
    var msgInt16 = rclnodejs.require('std_msgs').msg.Int16;
    var msg = new msgInt16();

    msg.data = 0x7fff;
    assert.deepStrictEqual(typeof msg.data, 'number');
    assert.deepStrictEqual(msg.data, 0x7fff);

    assertThrowsError(
      () => {
        msg.data = -0x7fff - 2;
      },
      [TypeError, RangeError],
      'out of bounds',
      'Int16 should be in [-0x7fff, 0x7fff]'
    );
    assertThrowsError(
      () => {
        msg.data = 0x7ffff + 1;
      },
      [TypeError, RangeError],
      'out of bounds',
      'Int16 should be in [-0x7fff, 0x7fff]'
    );
  });

  it('UInt16 data checking', function() {
    var node = rclnodejs.createNode('uint16');
    var msgUInt16 = rclnodejs.require('std_msgs').msg.UInt16;
    var msg = new msgUInt16();

    msg.data = 0xffff;
    assert.deepStrictEqual(typeof msg.data, 'number');
    assert.deepStrictEqual(msg.data, 0xffff);

    assertThrowsError(
      () => {
        msg.data = -1;
      },
      [TypeError, RangeError],
      'out of bounds',
      'UInt16 should be in [0, 0xffff]'
    );
    assertThrowsError(
      () => {
        msg.data = 0xffff + 1;
      },
      [TypeError, RangeError],
      'out of bounds',
      'UInt16 should be in [0, 0xffff]'
    );
  });

  it('Int32 data checking', function() {
    var node = rclnodejs.createNode('int32');
    var msgInt32 = rclnodejs.require('std_msgs').msg.Int32;
    var msg = new msgInt32();

    msg.data = 0x7fffffff;
    assert.deepStrictEqual(typeof msg.data, 'number');
    assert.deepStrictEqual(msg.data, 0x7fffffff);

    assertThrowsError(
      () => {
        msg.data = -0x7fffffff - 2;
      },
      [TypeError, RangeError],
      'out of bounds',
      'Int32 should be in [-0x7fffffff - 1, 0x7fffffff]'
    );
    assertThrowsError(
      () => {
        msg.data = 0x7ffffffff + 1;
      },
      [TypeError, RangeError],
      'out of bounds',
      'Int32 should be in [-0x7fffffff - 1, 0x7fffffff]'
    );
  });

  it('UInt32 data checking', function() {
    var node = rclnodejs.createNode('uint32');
    var msgUInt32 = rclnodejs.require('std_msgs').msg.UInt32;
    var msg = new msgUInt32();

    msg.data = 0xffffffff;
    assert.deepStrictEqual(typeof msg.data, 'number');
    assert.deepStrictEqual(msg.data, 0xffffffff);

    assertThrowsError(
      () => {
        msg.data = -1;
      },
      [TypeError, RangeError],
      'out of bounds',
      'UInt32 should be in [0, 0xffffffff]'
    );
    assertThrowsError(
      () => {
        msg.data = 0xffffffff + 1;
      },
      [TypeError, RangeError],
      'out of bounds',
      'UInt32 should be in [0, 0xffffffff]'
    );
  });

  it('Int64 data checking', function() {
    var node = rclnodejs.createNode('int64');
    var msgInt64 = rclnodejs.require('std_msgs').msg.Int64;
    var msg = new msgInt64();

    msg.data = 0x1fffffffffffff;
    assert.deepStrictEqual(typeof msg.data, 'number');
    assert.deepStrictEqual(msg.data, Number.MAX_SAFE_INTEGER);
  });

  it('UInt64 data checking', function() {
    var node = rclnodejs.createNode('uint64');
    var msgUInt64 = rclnodejs.require('std_msgs').msg.UInt64;
    var msg = new msgUInt64();

    msg.data = 0x1fffffffffffff;
    assert.deepStrictEqual(typeof msg.data, 'number');
    assert.deepStrictEqual(msg.data, Number.MAX_SAFE_INTEGER);
  });

  it('Float32 data checking', function() {
    var node = rclnodejs.createNode('float32');
    var msgFloat32 = rclnodejs.require('std_msgs').msg.Float32;
    var msg = new msgFloat32();

    msg.data = 3.14;
    assert.deepStrictEqual(typeof msg.data, 'number');
    assert.ok(Math.abs(msg.data - 3.14) < 0.000001);
  });

  it('Float64 data checking', function() {
    var node = rclnodejs.createNode('float64');
    var msgFloat64 = rclnodejs.require('std_msgs').msg.Float64;
    var msg = new msgFloat64();

    msg.data = 3.14;
    assert.deepStrictEqual(typeof msg.data, 'number');
    assert.ok(Math.abs(msg.data - 3.14) < 0.000001);
  });
});
