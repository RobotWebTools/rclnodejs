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
const { Clock, ROSClock, ClockType, Duration, Time } = rclnodejs;

describe('Clock sleep methods', function () {
  this.timeout(15000);

  before(async function () {
    await rclnodejs.init();
  });

  after(function () {
    rclnodejs.shutdown();
  });

  describe('sleepFor', function () {
    it('should sleep for the specified duration with STEADY_TIME', async function () {
      const clock = new Clock(ClockType.STEADY_TIME);
      const duration = new Duration(0n, 200000000n); // 200 milliseconds

      const start = Date.now();
      const result = await clock.sleepFor(duration);
      const elapsed = Date.now() - start;

      assert.strictEqual(result, true, 'Sleep should return true');
      assert.ok(
        elapsed >= 190 && elapsed <= 250,
        `Elapsed time ${elapsed}ms should be approximately 200ms`
      );
    });

    it('should sleep for the specified duration with SYSTEM_TIME', async function () {
      const clock = new Clock(ClockType.SYSTEM_TIME);
      const duration = new Duration(0n, 150000000n); // 150 milliseconds

      const start = Date.now();
      const result = await clock.sleepFor(duration);
      const elapsed = Date.now() - start;

      assert.strictEqual(result, true, 'Sleep should return true');
      assert.ok(
        elapsed >= 140 && elapsed <= 200,
        `Elapsed time ${elapsed}ms should be approximately 150ms`
      );
    });

    it('should sleep for the specified duration with ROSClock', async function () {
      const clock = new ROSClock();
      const duration = new Duration(0n, 100000000n); // 100 milliseconds

      const start = Date.now();
      const result = await clock.sleepFor(duration);
      const elapsed = Date.now() - start;

      assert.strictEqual(result, true, 'Sleep should return true');
      assert.ok(
        elapsed >= 90 && elapsed <= 150,
        `Elapsed time ${elapsed}ms should be approximately 100ms`
      );
    });

    it('should sleep for 1 second', async function () {
      const clock = new Clock(ClockType.STEADY_TIME);
      const duration = new Duration(1n, 0n); // 1 second

      const startTime = clock.now();
      const result = await clock.sleepFor(duration);
      const endTime = clock.now();

      const elapsed = endTime.nanoseconds - startTime.nanoseconds;
      assert.strictEqual(result, true, 'Sleep should return true');
      assert.ok(
        elapsed >= 950000000n && elapsed <= 1100000000n,
        `Elapsed time should be approximately 1 second, got ${Number(elapsed) / 1e9}s`
      );
    });
  });

  describe('sleepUntil', function () {
    it('should sleep until the specified time with STEADY_TIME', async function () {
      const clock = new Clock(ClockType.STEADY_TIME);
      const currentTime = clock.now();
      const targetTime = new Time(
        0n,
        currentTime.nanoseconds + 200000000n, // +200ms
        ClockType.STEADY_TIME
      );

      const start = Date.now();
      const result = await clock.sleepUntil(targetTime);
      const elapsed = Date.now() - start;

      assert.strictEqual(result, true, 'Sleep should return true');
      assert.ok(
        elapsed >= 190 && elapsed <= 250,
        `Elapsed time ${elapsed}ms should be approximately 200ms`
      );
    });

    it('should sleep until the specified time with SYSTEM_TIME', async function () {
      const clock = new Clock(ClockType.SYSTEM_TIME);
      const currentTime = clock.now();
      const targetTime = new Time(
        0n,
        currentTime.nanoseconds + 150000000n, // +150ms
        ClockType.SYSTEM_TIME
      );

      const start = Date.now();
      const result = await clock.sleepUntil(targetTime);
      const elapsed = Date.now() - start;

      assert.strictEqual(result, true, 'Sleep should return true');
      assert.ok(
        elapsed >= 140 && elapsed <= 200,
        `Elapsed time ${elapsed}ms should be approximately 150ms`
      );
    });

    it('should throw TypeError for mismatched clock types', async function () {
      const clock = new Clock(ClockType.STEADY_TIME);
      const targetTime = new Time(0n, 1000000000n, ClockType.SYSTEM_TIME);

      try {
        await clock.sleepUntil(targetTime);
        assert.fail('Should have thrown TypeError');
      } catch (err) {
        assert.ok(err instanceof TypeError);
        assert.ok(
          err.message.includes('clock type'),
          'Error message should mention clock type mismatch'
        );
      }
    });

    it('should sleep until a time in the past (return immediately)', async function () {
      const clock = new Clock(ClockType.STEADY_TIME);
      const currentTime = clock.now();
      // Set target time in the past
      const targetTime = new Time(
        0n,
        currentTime.nanoseconds - 1000000000n, // -1 second
        ClockType.STEADY_TIME
      );

      const start = Date.now();
      const result = await clock.sleepUntil(targetTime);
      const elapsed = Date.now() - start;

      // Should return almost immediately since target is in the past
      assert.ok(
        elapsed < 50,
        `Should return quickly for past time, took ${elapsed}ms`
      );
      // Result depends on implementation - it reached the target time immediately
      assert.ok(result === true || result === false);
    });
  });

  describe('error handling', function () {
    it('should throw error for invalid context', async function () {
      const clock = new Clock(ClockType.STEADY_TIME);
      const Context = require('../lib/context.js');
      const invalidContext = new Context();
      // Don't initialize the context

      const duration = new Duration(0n, 100000000n);

      try {
        await clock.sleepFor(duration, invalidContext);
        assert.fail('Should have thrown error for invalid context');
      } catch (err) {
        assert.ok(
          err.message.includes('Context') ||
            err.message.includes('initialized') ||
            err.message.includes('shutdown')
        );
      }
    });
  });

  describe('integration with ROSClock', function () {
    it('should work with ROSClock when ROS time is inactive', async function () {
      const clock = new ROSClock();
      // Ensure ROS time is not active (should behave like system time)
      clock.isRosTimeActive = false;

      const duration = new Duration(0n, 100000000n); // 100ms

      const start = Date.now();
      const result = await clock.sleepFor(duration);
      const elapsed = Date.now() - start;

      assert.strictEqual(result, true);
      assert.ok(
        elapsed >= 90 && elapsed <= 150,
        `Elapsed time ${elapsed}ms should be approximately 100ms`
      );
    });

    it('should work with ROSClock when ROS time is active', async function () {
      // Create a node with use_sim_time parameter set to true
      const options = new rclnodejs.NodeOptions();
      options.parameterOverrides.push(
        new rclnodejs.Parameter(
          'use_sim_time',
          rclnodejs.ParameterType.PARAMETER_BOOL,
          true
        )
      );

      const node = rclnodejs.createNode(
        'TestClockSleepRosTime',
        '',
        rclnodejs.Context.defaultContext(),
        options
      );

      try {
        // Set up TimeSource to activate ROS time
        const TimeSource = require('../lib/time_source.js');
        const timeSource = new TimeSource(node);
        const clock = new ROSClock();
        timeSource.attachClock(clock);

        // Start spinning the node to receive clock messages
        rclnodejs.spin(node);

        // Verify ROS time is active
        assert.strictEqual(clock.isRosTimeActive, true);

        // Create a clock publisher to simulate time
        const clockPub = node.createPublisher(
          'rosgraph_msgs/msg/Clock',
          '/clock'
        );

        // Publish initial time: 10 seconds
        const startSec = 10;
        clockPub.publish({ clock: { sec: startSec, nanosec: 0 } });

        // Wait a bit for the message to be processed
        await new Promise((resolve) => setTimeout(resolve, 100));

        // Get current ROS time (should be 10 seconds)
        const currentRosTime = clock.now();
        assert.ok(
          currentRosTime.nanoseconds >= BigInt(startSec) * 1000000000n,
          'ROS time should be at least 10 seconds'
        );

        // Test sleepFor with simulated time
        // We'll publish clock messages while sleeping to advance time
        const sleepDuration = new Duration(2n, 0n); // 2 seconds of ROS time

        const sleepPromise = clock.sleepFor(sleepDuration);

        // Publish clock messages to advance time
        let currentSimTime = startSec;
        const publishInterval = setInterval(() => {
          currentSimTime += 1; // Advance by 1 second each time
          clockPub.publish({ clock: { sec: currentSimTime, nanosec: 0 } });
        }, 100); // Publish every 100ms wall time

        // Wait for sleep to complete
        const result = await sleepPromise;
        clearInterval(publishInterval);

        assert.strictEqual(result, true, 'Sleep should complete successfully');

        // Verify that ROS time advanced
        const finalRosTime = clock.now();
        assert.ok(
          finalRosTime.nanoseconds >=
            currentRosTime.nanoseconds + sleepDuration.nanoseconds,
          'ROS time should have advanced by at least the sleep duration'
        );
      } finally {
        node.destroy();
      }
    });
  });
});
