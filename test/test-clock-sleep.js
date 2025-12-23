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
  });
});
