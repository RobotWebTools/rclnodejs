// Copyright (c) 2025, The Robot Web Tools Contributors
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
const rclnodejsNative = require('../lib/native_loader.js');
const { Time, Clock } = rclnodejs;
const { ClockType } = Clock;

describe('rclnodejs Clock Callback testing', function () {
  this.timeout(60 * 1000);

  before(function () {
    return rclnodejs.init();
  });

  after(function () {
    rclnodejs.shutdown();
  });

  let testClock = null;
  let testCallbacks = [];

  afterEach(function () {
    // Clean up all registered callbacks
    if (testClock && testCallbacks.length > 0) {
      testCallbacks.forEach((callback) => {
        try {
          testClock.removeClockCallback(callback);
        } catch (e) {
          // Callback may already be removed
        }
      });
    }
    testClock = null;
    testCallbacks = [];
  });

  it('Test add and remove clock callback', async function () {
    const clock = new Clock(ClockType.ROS_TIME);
    testClock = clock;

    let preCallbackCalled = false;
    let postCallbackCalled = false;
    let lastJumpInfo = null;

    const callbackObj = {
      _pre_callback: function () {
        preCallbackCalled = true;
      },
      _post_callback: function (jumpInfo) {
        postCallbackCalled = true;
        lastJumpInfo = jumpInfo;
      },
    };

    // Add callback
    clock.addClockCallback(callbackObj, true, 0n, 0n);
    testCallbacks.push(callbackObj);

    // Enable ROS time override
    rclnodejsNative.setRosTimeOverrideIsEnabled(clock.handle, true);

    // Create a time point for override
    const timePoint = new Time(100n, 0n, ClockType.ROS_TIME);

    // Set override (should trigger jump)
    rclnodejsNative.setRosTimeOverride(clock.handle, timePoint._handle);

    // Wait for callbacks to be processed
    await new Promise((resolve) => setImmediate(resolve));

    assert.strictEqual(
      preCallbackCalled,
      true,
      'Pre callback should be called'
    );
    assert.strictEqual(
      postCallbackCalled,
      true,
      'Post callback should be called'
    );
    assert.ok(lastJumpInfo, 'Jump info should be present');
    assert.strictEqual(typeof lastJumpInfo.clock_change, 'number');
    assert.strictEqual(typeof lastJumpInfo.delta, 'bigint');

    // Reset flags
    preCallbackCalled = false;
    postCallbackCalled = false;
    lastJumpInfo = null;

    // Remove callback
    clock.removeClockCallback(callbackObj);

    // Trigger another jump
    const timePoint2 = new Time(200n, 0n, ClockType.ROS_TIME);
    rclnodejsNative.setRosTimeOverride(clock.handle, timePoint2._handle);

    // Wait for any potential callbacks
    await new Promise((resolve) => setImmediate(resolve));

    assert.strictEqual(
      preCallbackCalled,
      false,
      'Pre callback should NOT be called after removal'
    );
    assert.strictEqual(
      postCallbackCalled,
      false,
      'Post callback should NOT be called after removal'
    );
  });

  it('Test clock callback thresholds', async function () {
    const clock = new Clock(ClockType.ROS_TIME);
    testClock = clock;
    let callbackCalled = false;

    const callbackObj = {
      _pre_callback: function () {},
      _post_callback: function (jumpInfo) {
        // console.log('Callback called with delta:', jumpInfo.delta);
        callbackCalled = true;
      },
    };

    // Add callback with threshold of 1 second (10^9 nanoseconds)
    // Set onClockChange to false to ensure we are testing time jump threshold
    clock.addClockCallback(callbackObj, false, 1000000000n, 0n);
    testCallbacks.push(callbackObj);

    rclnodejsNative.setRosTimeOverrideIsEnabled(clock.handle, true);

    // Jump forward by 0.5 seconds (should NOT trigger)
    let timePoint = new Time(0n, 500000000n, ClockType.ROS_TIME);
    rclnodejsNative.setRosTimeOverride(clock.handle, timePoint._handle);

    // Wait for callbacks to be processed
    await new Promise((resolve) => setImmediate(resolve));

    assert.strictEqual(
      callbackCalled,
      false,
      'Callback should not be called for small jump'
    );

    // Jump forward by 1.5 seconds (total 2.0s) (should trigger)
    // Current time is 0.5s. New time is 2.0s. Delta is 1.5s > 1.0s.
    timePoint = new Time(2n, 0n, ClockType.ROS_TIME);
    rclnodejsNative.setRosTimeOverride(clock.handle, timePoint._handle);

    // Wait for callbacks to be processed
    await new Promise((resolve) => setImmediate(resolve));

    assert.strictEqual(
      callbackCalled,
      true,
      'Callback should be called for large jump'
    );
  });

  it('Test multiple clock callbacks', async function () {
    const clock = new Clock(ClockType.ROS_TIME);
    testClock = clock;
    let callback1Called = false;
    let callback2Called = false;

    const callbackObj1 = {
      _pre_callback: () => {},
      _post_callback: () => {
        callback1Called = true;
      },
    };

    const callbackObj2 = {
      _pre_callback: () => {},
      _post_callback: () => {
        callback2Called = true;
      },
    };

    clock.addClockCallback(callbackObj1, true, 0n, 0n);
    testCallbacks.push(callbackObj1);
    clock.addClockCallback(callbackObj2, true, 0n, 0n);
    testCallbacks.push(callbackObj2);

    rclnodejsNative.setRosTimeOverrideIsEnabled(clock.handle, true);
    const timePoint = new Time(1n, 0n, ClockType.ROS_TIME);
    rclnodejsNative.setRosTimeOverride(clock.handle, timePoint._handle);

    // Wait for callbacks to be processed
    await new Promise((resolve) => setImmediate(resolve));

    assert.strictEqual(callback1Called, true);
    assert.strictEqual(callback2Called, true);
  });
});
