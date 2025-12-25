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

const rclnodejs = require('./native_loader.js');

/**
 * @class - Class representing a ClockEvent in ROS
 */
class ClockEvent {
  constructor() {
    this._handle = rclnodejs.createClockEvent();
  }

  /**
   * Wait until a time specified by a steady clock.
   * @param {Clock} clock - The clock to use for time synchronization.
   * @param {bigint} until - The time to wait until.
   * @return {Promise<void>} - A promise that resolves when the time is reached.
   */
  async waitUntilSteady(clock, until) {
    return rclnodejs.clockEventWaitUntilSteady(
      this._handle,
      clock.handle,
      until
    );
  }

  /**
   * Wait until a time specified by a system clock.
   * @param {Clock} clock - The clock to use for time synchronization.
   * @param {bigint} until - The time to wait until.
   * @return {Promise<void>} - A promise that resolves when the time is reached.
   */
  async waitUntilSystem(clock, until) {
    return rclnodejs.clockEventWaitUntilSystem(
      this._handle,
      clock.handle,
      until
    );
  }

  /**
   * Wait until a time specified by a ROS clock.
   * @param {Clock} clock - The clock to use for time synchronization.
   * @param {bigint} until - The time to wait until.
   * @return {Promise<void>} - A promise that resolves when the time is reached.
   */
  async waitUntilRos(clock, until) {
    return rclnodejs.clockEventWaitUntilRos(this._handle, clock.handle, until);
  }

  /**
   * Indicate if the ClockEvent is set.
   * @return {boolean} - True if the ClockEvent is set.
   */
  isSet() {
    return rclnodejs.clockEventIsSet(this._handle);
  }

  /**
   * Set the event.
   */
  set() {
    rclnodejs.clockEventSet(this._handle);
  }

  /**
   * Clear the event.
   */
  clear() {
    rclnodejs.clockEventClear(this._handle);
  }
}

module.exports = ClockEvent;
