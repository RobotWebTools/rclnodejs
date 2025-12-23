'use strict';

const assert = require('assert');
const rclnodejs = require('../index.js');
const { Clock, ROSClock, ClockType, ClockChange, Duration, Time } = rclnodejs;

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
