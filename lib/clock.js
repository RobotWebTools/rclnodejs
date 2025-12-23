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

const rclnodejs = require('./native_loader.js');
const Time = require('./time.js');
const ClockType = require('./clock_type.js');
const ClockChange = require('./clock_change.js');
const Context = require('./context.js');
const ClockEvent = require('./clock_event.js');
const { TypeValidationError } = require('./errors.js');

/**
 * @class - Class representing a Clock in ROS
 */

class Clock {
  /**
   * Create a Clock.
   * @param {ClockType} [clockType=Clock.ClockType.SYSTEM_TIME] - The type of the clock to be created.
   */
  constructor(clockType = ClockType.SYSTEM_TIME) {
    this._clockType = clockType;
    this._handle = rclnodejs.createClock(this._clockType);
  }

  /**
   * @ignore
   */
  get handle() {
    return this._handle;
  }

  /**
   * Get ClockType of this Clock object.
   * @return {ClockType} Return the type of the clock.
   */
  get clockType() {
    return this._clockType;
  }

  /**
   * Add a clock callback.
   * @param {object} callbackObject - The object containing callback methods.
   * @param {Function} [callbackObject._pre_callback] - Optional callback invoked before a time jump. Takes no arguments.
   * @param {Function} [callbackObject._post_callback] - Optional callback invoked after a time jump.
   *   Receives a jumpInfo object with properties:
   *   - clock_change {number}: Type of clock change that occurred.
   *   - delta {bigint}: Time delta in nanoseconds.
   * @param {boolean} onClockChange - Whether to call the callback on clock change.
   * @param {bigint} minForward - Minimum forward jump in nanoseconds to trigger the callback.
   * @param {bigint} minBackward - Minimum backward jump in nanoseconds to trigger the callback.
   */
  addClockCallback(callbackObject, onClockChange, minForward, minBackward) {
    if (typeof callbackObject !== 'object' || callbackObject === null) {
      throw new TypeValidationError(
        'callbackObject',
        callbackObject,
        'object',
        {
          entityType: 'clock',
        }
      );
    }
    if (typeof onClockChange !== 'boolean') {
      throw new TypeValidationError('onClockChange', onClockChange, 'boolean', {
        entityType: 'clock',
      });
    }
    if (typeof minForward !== 'bigint') {
      throw new TypeValidationError('minForward', minForward, 'bigint', {
        entityType: 'clock',
      });
    }
    if (typeof minBackward !== 'bigint') {
      throw new TypeValidationError('minBackward', minBackward, 'bigint', {
        entityType: 'clock',
      });
    }
    rclnodejs.clockAddJumpCallback(
      this._handle,
      callbackObject,
      onClockChange,
      minForward,
      minBackward
    );
  }

  /**
   * Remove a clock callback.
   * @param {object} callbackObject - The callback object that was previously registered with addClockCallback().
   */
  removeClockCallback(callbackObject) {
    if (typeof callbackObject !== 'object' || callbackObject === null) {
      throw new TypeValidationError(
        'callbackObject',
        callbackObject,
        'object',
        {
          entityType: 'clock',
        }
      );
    }
    rclnodejs.clockRemoveJumpCallback(this._handle, callbackObject);
  }

  /**
   * Return the current time.
   * @return {Time} Return the current time.
   */
  now() {
    const nowInNanosec = rclnodejs.clockGetNow(this._handle);
    return new Time(0n, nowInNanosec, this._clockType);
  }

  /**
   * Sleep until a specific time is reached on this Clock.
   *
   * When using a ROSClock, this may sleep forever if the TimeSource is misconfigured
   * and the context is never shut down. ROS time being activated or deactivated causes
   * this function to cease sleeping and return false.
   *
   * @param {Time} until - Time at which this function should stop sleeping.
   * @param {Context} [context=null] - Context whose validity will be checked while sleeping.
   *   If context is null, then the default context is used. If the context is found to be
   *   shut down before or by the time the wait completes, the returned promise resolves to false.
   * @return {Promise<boolean>} Promise that resolves to true if 'until' was reached without
   *   detecting a time jump or a shut down context, or false otherwise (for example, if a time
   *   jump occurred or the context is no longer valid when the wait completes).
   * @throws {TypeError} if until is specified for a different type of clock than this one.
   * @throws {Error} if context has not been initialized or is shutdown.
   */
  async sleepUntil(until, context = null) {
    if (!(until instanceof Time)) {
      throw new TypeValidationError('until', until, 'Time', {
        entityType: 'clock',
      });
    }

    if (!context) {
      context = Context.defaultContext();
    }

    if (!context.isValid()) {
      throw new Error('Context is not initialized or has been shut down');
    }

    if (until.clockType !== this._clockType) {
      throw new TypeError(
        "until's clock type does not match this clock's type"
      );
    }

    const event = new ClockEvent();
    let timeSourceChanged = false;

    // Callback to wake when time jumps and is past target time
    const callbackObject = {
      _pre_callback: () => {
        // Optional pre-callback - no action needed
      },
      _post_callback: (jumpInfo) => {
        // ROS time being activated or deactivated changes the epoch,
        // so sleep time loses its meaning
        timeSourceChanged =
          timeSourceChanged ||
          jumpInfo.clock_change === ClockChange.ROS_TIME_ACTIVATED ||
          jumpInfo.clock_change === ClockChange.ROS_TIME_DEACTIVATED;

        if (timeSourceChanged || this.now().nanoseconds >= until.nanoseconds) {
          event.set();
        }
      },
    };

    // Setup jump callback with minimal forward threshold
    this.addClockCallback(
      callbackObject,
      true, // onClockChange
      1n, // minForward (1 nanosecond - any forward jump triggers)
      0n // minBackward (0 disables backward threshold)
    );

    try {
      // Wait based on clock type
      if (this._clockType === ClockType.SYSTEM_TIME) {
        await event.waitUntilSystem(this, until.nanoseconds);
      } else if (this._clockType === ClockType.STEADY_TIME) {
        await event.waitUntilSteady(this, until.nanoseconds);
      } else if (this._clockType === ClockType.ROS_TIME) {
        await event.waitUntilRos(this, until.nanoseconds);
      }
    } finally {
      // Always clean up callback
      this.removeClockCallback(callbackObject);
    }

    if (!context.isValid() || timeSourceChanged) {
      return false;
    }

    return this.now().nanoseconds >= until.nanoseconds;
  }

  /**
   * Sleep for a specified duration.
   *
   * Equivalent to: clock.sleepUntil(clock.now() + duration, context)
   *
   * When using a ROSClock, this may sleep forever if the TimeSource is misconfigured
   * and the context is never shut down. ROS time being activated or deactivated causes
   * this function to cease sleeping and return false.
   *
   * @param {Duration} duration - Duration of time to sleep for.
   * @param {Context} [context=null] - Context which when shut down will cause this sleep to wake early.
   *   If context is null, then the default context is used.
   * @return {Promise<boolean>} Promise that resolves to true if the full duration was slept,
   *   or false if it woke for another reason.
   * @throws {Error} if context has not been initialized or is shutdown.
   */
  async sleepFor(duration, context = null) {
    const until = this.now().add(duration);
    return this.sleepUntil(until, context);
  }
}

/**
 * @class - Class representing a ROSClock in ROS
 */

class ROSClock extends Clock {
  /**
   * Create a ROSClock.
   */
  constructor() {
    super(ClockType.ROS_TIME);
  }

  /**
   * Return status that whether the ROS time is active.
   * @name ROSClock#get:isRosTimeActive
   * @function
   * @return {boolean} Return true if the time is active, otherwise return false.
   */

  get isRosTimeActive() {
    return rclnodejs.getRosTimeOverrideIsEnabled(this._handle);
  }

  /**
   * Set the status of ROS time.
   * @param {boolean} enabled - Set the ROS time to be active.
   * @name ROSClock#set:isRosTimeActive
   * @function
   * @return {undefined}
   */

  set isRosTimeActive(enabled) {
    rclnodejs.setRosTimeOverrideIsEnabled(this._handle, enabled);
  }

  /**
   * Set the status of ROS time.
   * @param {Time} time - The time to be set override.
   * @name ROSClock#set:rosTimeOverride
   * @function
   * @return {undefined}
   */

  set rosTimeOverride(time) {
    if (!(time instanceof Time)) {
      throw new TypeValidationError('time', time, 'Time', {
        entityType: 'clock',
      });
    }
    rclnodejs.setRosTimeOverride(this._handle, time._handle);
  }
}

Clock.ClockType = ClockType;

module.exports = { Clock, ROSClock };
