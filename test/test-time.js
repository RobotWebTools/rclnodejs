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
const {Time, Clock, Duration} = rclnodejs;

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
    assert.strictEqual(time.clockType, Clock.ClockType.SYSTEM_TIME);

    assert.throws(() => {
      new Time(0, Number.MAX_SAFE_INTEGER + 1);
    }, RangeError);

    assert.throws(() => {
      new Time(1, 1, 'SYSTEM_TIME');
    }, TypeError);

    assert.throws(() => {
      new Time(-1, 0);
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

    assert.throws(() => {
      new Duration(0, Number.MAX_SAFE_INTEGER + 1);
    }, RangeError);
  });

  it('Test time functions', function() {
    let left = new Time(0, 1);
    let right = new Time(0, 2);

    assert.strictEqual(left.isEqual(right), false);
    assert.strictEqual(left.isNotEqual(right), true);
    assert.strictEqual(left.isLessThan(right), true);
    assert.strictEqual(left.isLessThanOrEqual(right), true);
    assert.strictEqual(left.isGreaterThan(right), false);
    assert.strictEqual(left.isGreaterThanOrEqual(right), false);

    left = new Time(0, 1, Clock.ClockType.SYSTEM_TIME);
    right = new Time(0, 2, Clock.ClockType.STEADY_TIME);

    assert.throws(() => {
      left.isEqual(right);
    }, TypeError);

    assert.throws(() => {
      left.isNotEqual(right);
    }, TypeError);

    assert.throws(() => {
      left.isLessThan(right);
    }, TypeError);

    assert.throws(() => {
      left.isLessThanOrEqual(right);
    }, TypeError);

    assert.throws(() => {
      left.isGreaterThan(right);
    }, TypeError);

    assert.throws(() => {
      left.isGreaterThanOrEqual(right);
    }, TypeError);

    let time = new Time(0, 1, Clock.ClockType.STEADY_TIME);
    let duration = new Duration(0, 1);
    let result = time.add(duration);
    assert.ok(result instanceof Time);
    assert.strictEqual(result.nanoseconds, 2);
    assert.strictEqual(result.clockType, Clock.ClockType.STEADY_TIME);

    result = time.sub(duration);
    assert.ok(result instanceof Time);
    assert.strictEqual(result.nanoseconds, 0);
    assert.strictEqual(result.clockType, Clock.ClockType.STEADY_TIME);

    let diff = time.sub(result);
    assert.ok(diff instanceof Duration);
    assert.strictEqual(diff.nanoseconds, 1);
    assert.throws(() => {
      time.add(result);
    }, TypeError);
  });

  it('Test duration functions', function() {
    let left = new Duration(0, 1);
    let right = new Duration(0, 2);
    assert.strictEqual(left.isEqual(right), false);
    assert.strictEqual(left.isNotEqual(right), true);
    assert.strictEqual(left.isGreaterThan(right), false);
    assert.strictEqual(left.isGreaterThanOrEqual(right), false);
    assert.strictEqual(left.isLessThan(right), true);
    assert.strictEqual(left.isLessThanOrEqual(right), true);

    left = new Duration(0, 5e9);
    right = new Duration(5, 0);
    assert.ok(left.isEqual(right));

    assert.throws(() => {
      left.isEqual(5e9);
    }, TypeError);

    let time = new Time();
    assert.throws(() => {
      left.isEqual(time);
    }, TypeError);
    assert.throws(() => {
      left.isNotEqual(time);
    }, TypeError);
    assert.throws(() => {
      left.isGreaterThan(time);
    }, TypeError);
    assert.throws(() => {
      left.isGreaterThanOrEqual(time);
    }, TypeError);
    assert.throws(() => {
      left.isLessThan(time);
    }, TypeError);
    assert.throws(() => {
      left.isLessThanOrEqual(time);
    }, TypeError);
  });
});
