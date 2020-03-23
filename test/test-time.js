// Copyright (c) 2018 Intel Corporation. All rights reserved.
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
const { Time, Clock, Duration } = rclnodejs;
const { ClockType } = Clock;

describe('rclnodejs Time/Clock testing', function() {
  this.timeout(60 * 1000);

  before(function() {
    return rclnodejs.init();
  });

  after(function() {
    rclnodejs.shutdown();
  });

  it('Construct time object', function() {
    let time = new Time(1, 64);
    assert.strictEqual(time.nanoseconds, 1000000064);
    assert.strictEqual(time.clockType, ClockType.SYSTEM_TIME);

    time = new Time(0, Number.MAX_SAFE_INTEGER);
    assert.strictEqual(time.nanoseconds, 9007199254740991);

    // The nanoseconds property will be presented in a string, if the value excesses 2^53-1
    time = new Time(0, '9007199254740992');
    assert.strictEqual(time.nanoseconds, '9007199254740992');

    time = new Time(0, '9223372036854775807');
    assert.strictEqual(time.nanoseconds, '9223372036854775807');

    time = Time.fromMsg({ sec: 1, nanosec: 64 });
    assert.strictEqual(time.nanoseconds, 1000000064);
    assert.strictEqual(time.clockType, ClockType.ROS_TIME);

    assert.throws(() => {
      new Time(1, 1, 'SYSTEM_TIME');
    }, TypeError);

    assert.throws(() => {
      new Time({ seconds: 0, nanoseconds: 0 });
    }, TypeError);

    assert.throws(() => {
      new Time(-1, 0);
    }, RangeError);

    assert.throws(() => {
      new Time(0, '-9007199254740992');
    }, RangeError);

    assert.throws(() => {
      new Time(0, -1);
    }, RangeError);
  });

  it('Construct duration object', function() {
    let duration = new Duration();
    assert.strictEqual(duration.nanoseconds, 0);

    duration = new Duration(1, 64);
    assert.strictEqual(duration.nanoseconds, 1000000064);

    duration = new Duration(-1);
    assert.strictEqual(duration.nanoseconds, -1000000000);

    duration = new Duration(0, -1);
    assert.strictEqual(duration.nanoseconds, -1);

    duration = new Duration(0, Number.MAX_SAFE_INTEGER);
    assert.strictEqual(duration.nanoseconds, 9007199254740991);

    duration = new Duration(0, '9007199254740992');
    assert.strictEqual(duration.nanoseconds, '9007199254740992');

    duration = new Duration(0, '-9007199254740992');
    assert.strictEqual(duration.nanoseconds, '-9007199254740992');

    duration = new Duration(0, '9223372036854775807');
    assert.strictEqual(duration.nanoseconds, '9223372036854775807');
  });

  it('Test time functions', function() {
    let left = new Time(0, 1);
    let right = new Time(0, 2);

    assert.strictEqual(left.eq(right), false);
    assert.strictEqual(left.ne(right), true);
    assert.strictEqual(left.lt(right), true);
    assert.strictEqual(left.lte(right), true);
    assert.strictEqual(left.gt(right), false);
    assert.strictEqual(left.gte(right), false);

    left = new Time(0, 1, ClockType.SYSTEM_TIME);
    right = new Time(0, 2, ClockType.STEADY_TIME);

    assert.throws(() => {
      left.eq(right);
    }, TypeError);

    assert.throws(() => {
      left.ne(right);
    }, TypeError);

    assert.throws(() => {
      left.lt(right);
    }, TypeError);

    assert.throws(() => {
      left.lte(right);
    }, TypeError);

    assert.throws(() => {
      left.gt(right);
    }, TypeError);

    assert.throws(() => {
      left.gte(right);
    }, TypeError);

    let time = new Time(0, 1, ClockType.STEADY_TIME);
    let duration = new Duration(0, 1);
    let result = time.add(duration);
    assert.ok(result instanceof Time);
    assert.strictEqual(result.nanoseconds, 2);
    assert.strictEqual(result.clockType, ClockType.STEADY_TIME);

    result = time.sub(duration);
    assert.ok(result instanceof Time);
    assert.strictEqual(result.nanoseconds, 0);
    assert.strictEqual(result.clockType, ClockType.STEADY_TIME);

    let diff = time.sub(result);
    assert.ok(diff instanceof Duration);
    assert.strictEqual(diff.nanoseconds, 1);
    assert.throws(() => {
      time.add(result);
    }, TypeError);

    let nanos = time._nanoseconds;
    time.secondsAndNanoseconds;
    assert.strictEqual(time._nanoseconds, nanos);
  });

  it('Test duration functions', function() {
    let left = new Duration(0, 1);
    let right = new Duration(0, 2);
    assert.strictEqual(left.eq(right), false);
    assert.strictEqual(left.ne(right), true);
    assert.strictEqual(left.gt(right), false);
    assert.strictEqual(left.gte(right), false);
    assert.strictEqual(left.lt(right), true);
    assert.strictEqual(left.lte(right), true);

    left = new Duration(0, 5e9);
    right = new Duration(5, 0);
    assert.ok(left.eq(right));

    assert.throws(() => {
      left.eq(5e9);
    }, TypeError);

    let time = new Time();
    assert.throws(() => {
      left.eq(time);
    }, TypeError);
    assert.throws(() => {
      left.ne(time);
    }, TypeError);
    assert.throws(() => {
      left.gt(time);
    }, TypeError);
    assert.throws(() => {
      left.gte(time);
    }, TypeError);
    assert.throws(() => {
      left.lt(time);
    }, TypeError);
    assert.throws(() => {
      left.lte(time);
    }, TypeError);
  });
});
