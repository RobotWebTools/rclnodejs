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

describe('rclnodejs Time/Clock testing', function () {
  this.timeout(60 * 1000);

  before(function () {
    return rclnodejs.init();
  });

  after(function () {
    rclnodejs.shutdown();
  });

  it('Construct time object', function () {
    let time = new Time(1n, 64n);
    assert.strictEqual(time.nanoseconds, 1000000064n);
    assert.strictEqual(time.clockType, ClockType.SYSTEM_TIME);

    time = new Time(0n, BigInt(Number.MAX_SAFE_INTEGER));
    assert.strictEqual(time.nanoseconds, 9007199254740991n);

    time = new Time(0n, 9007199254740992n);
    assert.strictEqual(time.nanoseconds, 9007199254740992n);

    time = new Time(0n, 9223372036854775807n);
    assert.strictEqual(time.nanoseconds, 9223372036854775807n);

    time = Time.fromMsg({ sec: 1n, nanosec: 64n });
    assert.strictEqual(time.nanoseconds, 1000000064n);
    assert.strictEqual(time.clockType, ClockType.ROS_TIME);

    assert.throws(() => {
      new Time(1n, 1n, 'SYSTEM_TIME');
    }, rclnodejs.TypeValidationError);

    assert.throws(() => {
      new Time({ seconds: 0n, nanoseconds: 0n });
    }, rclnodejs.TypeValidationError);

    assert.throws(() => {
      new Time(-1n, 0n);
    }, rclnodejs.RangeValidationError);

    assert.throws(() => {
      new Time(0n, -9007199254740992n);
    }, rclnodejs.RangeValidationError);

    assert.throws(() => {
      new Time(0n, -1n);
    }, rclnodejs.RangeValidationError);
  });

  it('Construct duration object', function () {
    let duration = new Duration();
    assert.strictEqual(duration.nanoseconds, 0n);

    duration = new Duration(1n, 64n);
    assert.strictEqual(duration.nanoseconds, 1000000064n);

    duration = new Duration(-1n);
    assert.strictEqual(duration.nanoseconds, -1000000000n);

    duration = new Duration(0n, -1n);
    assert.strictEqual(duration.nanoseconds, -1n);

    duration = new Duration(0n, BigInt(Number.MAX_SAFE_INTEGER));
    assert.strictEqual(duration.nanoseconds, 9007199254740991n);

    duration = new Duration(0n, 9007199254740992n);
    assert.strictEqual(duration.nanoseconds, 9007199254740992n);

    duration = new Duration(0n, -9007199254740992n);
    assert.strictEqual(duration.nanoseconds, -9007199254740992n);

    duration = new Duration(0n, 9223372036854775807n);
    assert.strictEqual(duration.nanoseconds, 9223372036854775807n);
  });

  it('Test time functions', function () {
    let left = new Time(0n, 1n);
    let right = new Time(0n, 2n);

    assert.strictEqual(left.eq(right), false);
    assert.strictEqual(left.ne(right), true);
    assert.strictEqual(left.lt(right), true);
    assert.strictEqual(left.lte(right), true);
    assert.strictEqual(left.gt(right), false);
    assert.strictEqual(left.gte(right), false);

    left = new Time(0n, 1n, ClockType.SYSTEM_TIME);
    right = new Time(0n, 2n, ClockType.STEADY_TIME);

    assert.throws(() => {
      left.eq(right);
    }, rclnodejs.TypeValidationError);

    assert.throws(() => {
      left.ne(right);
    }, rclnodejs.TypeValidationError);

    assert.throws(() => {
      left.lt(right);
    }, rclnodejs.TypeValidationError);

    assert.throws(() => {
      left.lte(right);
    }, rclnodejs.TypeValidationError);

    assert.throws(() => {
      left.gt(right);
    }, rclnodejs.TypeValidationError);

    assert.throws(() => {
      left.gte(right);
    }, rclnodejs.TypeValidationError);

    let time = new Time(0n, 1n, ClockType.STEADY_TIME);
    let duration = new Duration(0n, 1n);
    let result = time.add(duration);
    assert.ok(result instanceof Time);
    assert.strictEqual(result.nanoseconds, 2n);
    assert.strictEqual(result.clockType, ClockType.STEADY_TIME);

    result = time.sub(duration);
    assert.ok(result instanceof Time);
    assert.strictEqual(result.nanoseconds, 0n);
    assert.strictEqual(result.clockType, ClockType.STEADY_TIME);

    let diff = time.sub(result);
    assert.ok(diff instanceof Duration);
    assert.strictEqual(diff.nanoseconds, 1n);
    assert.throws(() => {
      time.add(result);
    }, rclnodejs.TypeValidationError);

    let nanos = time._nanoseconds;
    time.secondsAndNanoseconds;
    assert.strictEqual(time._nanoseconds, nanos);
  });

  it('Test duration functions', function () {
    let left = new Duration(0n, 1n);
    let right = new Duration(0n, 2n);
    assert.strictEqual(left.eq(right), false);
    assert.strictEqual(left.ne(right), true);
    assert.strictEqual(left.gt(right), false);
    assert.strictEqual(left.gte(right), false);
    assert.strictEqual(left.lt(right), true);
    assert.strictEqual(left.lte(right), true);

    left = new Duration(0n, 5n * 10n ** 9n);
    right = new Duration(5n, 0n);
    assert.ok(left.eq(right));

    assert.throws(() => {
      left.eq(5n ** 9n);
    }, rclnodejs.TypeValidationError);

    let time = new Time();
    assert.throws(() => {
      left.eq(time);
    }, rclnodejs.TypeValidationError);
    assert.throws(() => {
      left.ne(time);
    }, rclnodejs.TypeValidationError);
    assert.throws(() => {
      left.gt(time);
    }, rclnodejs.TypeValidationError);
    assert.throws(() => {
      left.gte(time);
    }, rclnodejs.TypeValidationError);
    assert.throws(() => {
      left.lt(time);
    }, rclnodejs.TypeValidationError);
    assert.throws(() => {
      left.lte(time);
    }, rclnodejs.TypeValidationError);
  });

  it('Conversion to Time message', function () {
    let time = new Time(100n, 200n);
    let msg = time.toMsg();

    assert.strictEqual(msg.sec, 100n);
    assert.strictEqual(msg.nanosec, 200n);
  });
});
