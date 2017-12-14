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

describe('Compound types', function() {
  this.timeout(60 * 1000);

  before(function() {
    return rclnodejs.init();
  });

  after(function() {
    rclnodejs.shutdown();
  });

  it('ColorRGBA', function() {
    const msgColorRGBA = rclnodejs.require('std_msgs').msg.ColorRGBA;
    let msg = new msgColorRGBA();

    assert.ok('r' in msg && 'g' in msg && 'b' in msg && 'a' in msg);
    assert.deepStrictEqual(typeof msg.r, 'number');
    assert.deepStrictEqual(typeof msg.g, 'number');
    assert.deepStrictEqual(typeof msg.b, 'number');
    assert.deepStrictEqual(typeof msg.a, 'number');
  });

  it('Array', function() {
    const Byte = rclnodejs.require('std_msgs').msg.Byte;
    const ByteArray = Byte.ArrayType;
    let msg = new ByteArray(3);
    msg.fill([1, 2, 3]);

    assert.deepStrictEqual(msg.data.length, 3);
    assert.deepStrictEqual(msg.data[0], 1);
    assert.deepStrictEqual(msg.data[1], 2);
    assert.deepStrictEqual(msg.data[2], 3);
  });

  it('Object with Header', function() {
    const Header = rclnodejs.require('std_msgs').msg.Header;
    let header = new Header();

    assert.ok('stamp' in header);
    assert.ok('frame_id' in header);

    assert.ok(typeof header.stamp, 'string');
    assert.ok(typeof header.frame_id, 'string');
  });

  it('Complex object', function() {
    const JointState = rclnodejs.require('sensor_msgs').msg.JointState;
    let state = new JointState();

    assert.ok('header' in state);
    assert.ok('name' in state);
    assert.ok('position' in state);
    assert.ok('velocity' in state);
    assert.ok('effort' in state);
  });
});
