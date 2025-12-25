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
const { Clock, ROSClock, ClockType, ClockChange, Time } = rclnodejs;

describe('ClockChange enum', function () {
  it('should define all clock change constants', function () {
    assert.strictEqual(typeof ClockChange, 'object');
    assert.strictEqual(typeof ClockChange.ROS_TIME_NO_CHANGE, 'number');
    assert.strictEqual(typeof ClockChange.ROS_TIME_ACTIVATED, 'number');
    assert.strictEqual(typeof ClockChange.ROS_TIME_DEACTIVATED, 'number');
    assert.strictEqual(typeof ClockChange.SYSTEM_TIME_NO_CHANGE, 'number');
  });

  it('should have correct numeric values', function () {
    assert.strictEqual(ClockChange.ROS_TIME_NO_CHANGE, 1);
    assert.strictEqual(ClockChange.ROS_TIME_ACTIVATED, 2);
    assert.strictEqual(ClockChange.ROS_TIME_DEACTIVATED, 3);
    assert.strictEqual(ClockChange.SYSTEM_TIME_NO_CHANGE, 4);
  });

  it('should match RCL constants', function () {
    // These values should match the C/C++ rcl_clock_change_t enum
    // RCL_ROS_TIME_NO_CHANGE = 1
    // RCL_ROS_TIME_ACTIVATED = 2
    // RCL_ROS_TIME_DEACTIVATED = 3
    // RCL_SYSTEM_TIME_NO_CHANGE = 4
    assert.strictEqual(ClockChange.ROS_TIME_NO_CHANGE, 1);
    assert.strictEqual(ClockChange.ROS_TIME_ACTIVATED, 2);
    assert.strictEqual(ClockChange.ROS_TIME_DEACTIVATED, 3);
    assert.strictEqual(ClockChange.SYSTEM_TIME_NO_CHANGE, 4);
  });

  it('should be exportable from rclnodejs module', function () {
    const { ClockChange: ImportedClockChange } = require('../index.js');
    assert.strictEqual(ImportedClockChange, ClockChange);
    assert.strictEqual(ImportedClockChange.ROS_TIME_ACTIVATED, 2);
  });

  it('should have all expected properties', function () {
    const expectedKeys = [
      'ROS_TIME_NO_CHANGE',
      'ROS_TIME_ACTIVATED',
      'ROS_TIME_DEACTIVATED',
      'SYSTEM_TIME_NO_CHANGE',
    ];

    const actualKeys = Object.keys(ClockChange);
    expectedKeys.forEach((key) => {
      assert.ok(
        actualKeys.includes(key),
        `ClockChange should have property ${key}`
      );
    });
  });

  it('should have distinct values', function () {
    const values = Object.values(ClockChange);
    const uniqueValues = new Set(values);
    assert.strictEqual(
      values.length,
      uniqueValues.size,
      'All ClockChange values should be unique'
    );
  });
});

describe('ClockChange integration with sleep methods', function () {
  this.timeout(5000);

  before(async function () {
    await rclnodejs.init();
  });

  after(function () {
    rclnodejs.shutdown();
  });

  it('should use ClockChange enum in clock jump callback', function (done) {
    const clock = new ROSClock();
    let callbackInvoked = false;
    let receivedClockChange = null;

    const callbackObj = {
      _pre_callback: () => {},
      _post_callback: (jumpInfo) => {
        callbackInvoked = true;
        receivedClockChange = jumpInfo.clock_change;

        // Verify the clock_change value matches ClockChange enum
        assert.ok(
          receivedClockChange === ClockChange.ROS_TIME_NO_CHANGE ||
            receivedClockChange === ClockChange.ROS_TIME_ACTIVATED ||
            receivedClockChange === ClockChange.ROS_TIME_DEACTIVATED ||
            receivedClockChange === ClockChange.SYSTEM_TIME_NO_CHANGE,
          'clock_change should be a valid ClockChange enum value'
        );
      },
    };

    // Add callback with threshold
    clock.addClockCallback(callbackObj, true, 0n, 0n);

    // Enable ROS time override to trigger a clock change
    clock.isRosTimeActive = true;

    // Set time to trigger callback
    const timePoint = new Time(100n, 0n, ClockType.ROS_TIME);
    clock.rosTimeOverride = timePoint;

    // Give it some time to process
    setTimeout(() => {
      clock.removeClockCallback(callbackObj);

      assert.ok(
        callbackInvoked,
        'Clock jump callback should have been invoked'
      );
      assert.ok(
        receivedClockChange !== null,
        'Should have received clock_change value'
      );

      // The change should be ROS_TIME_ACTIVATED since we enabled ROS time
      assert.strictEqual(
        receivedClockChange,
        ClockChange.ROS_TIME_ACTIVATED,
        'Clock change should be ROS_TIME_ACTIVATED'
      );

      done();
    }, 100);
  });

  it('should detect ROS_TIME_ACTIVATED in sleep callback', async function () {
    const clock = new ROSClock();
    clock.isRosTimeActive = false; // Start with ROS time disabled

    // Create a promise that will race with the sleep
    let detectedActivation = false;

    const callbackObj = {
      _pre_callback: () => {},
      _post_callback: (jumpInfo) => {
        if (jumpInfo.clock_change === ClockChange.ROS_TIME_ACTIVATED) {
          detectedActivation = true;
        }
      },
    };

    clock.addClockCallback(callbackObj, true, 0n, 0n);

    // Trigger ROS time activation after a short delay
    setTimeout(() => {
      clock.isRosTimeActive = true;
      const timePoint = new Time(200n, 0n, ClockType.ROS_TIME);
      clock.rosTimeOverride = timePoint;
    }, 50);

    // Wait a bit for callback to trigger
    await new Promise((resolve) => setTimeout(resolve, 150));

    clock.removeClockCallback(callbackObj);

    assert.ok(
      detectedActivation,
      'Should have detected ROS_TIME_ACTIVATED event'
    );
  });

  it('should use ClockChange constants for meaningful comparisons', function () {
    // Verify that the enum values are what we expect
    assert.strictEqual(
      ClockChange.ROS_TIME_ACTIVATED,
      2,
      'ROS_TIME_ACTIVATED should be 2'
    );
    assert.strictEqual(
      ClockChange.ROS_TIME_DEACTIVATED,
      3,
      'ROS_TIME_DEACTIVATED should be 3'
    );

    // Test that we can use the enum for comparisons
    const testValue = 2;
    assert.ok(
      testValue === ClockChange.ROS_TIME_ACTIVATED,
      'Should be able to compare numeric values with enum'
    );
  });

  it('should provide better code readability with ClockChange enum', function () {
    // This test demonstrates the improved readability
    const clockChangeValue = 2;

    // Old way (magic number):
    const isActivatedOldWay = clockChangeValue === 2;

    // New way (enum):
    const isActivatedNewWay =
      clockChangeValue === ClockChange.ROS_TIME_ACTIVATED;

    assert.strictEqual(
      isActivatedOldWay,
      isActivatedNewWay,
      'Both approaches should give same result'
    );

    // The enum approach is more self-documenting
    assert.ok(
      isActivatedNewWay,
      'Using ClockChange.ROS_TIME_ACTIVATED is more meaningful'
    );
  });

  it('ClockChange enum should be accessible via rclnodejs module', function () {
    const rclnodejs = require('../index.js');

    assert.ok(rclnodejs.ClockChange, 'ClockChange should be exported');
    assert.strictEqual(
      typeof rclnodejs.ClockChange,
      'object',
      'ClockChange should be an object'
    );
    assert.strictEqual(
      rclnodejs.ClockChange.ROS_TIME_ACTIVATED,
      2,
      'ROS_TIME_ACTIVATED should be accessible'
    );
  });
});
