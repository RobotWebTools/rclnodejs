// Copyright (c) 2017 Intel Corporation. All rights reserved.
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

const rclnodejs = require('bindings')('rclnodejs');

/**
 * @class - Class representing a Timer in ROS
 * @hideconstructor
 */

class Timer {
  constructor(handle, period, callback) {
    this._handle = handle;
    this._period = period;
    this.callback = callback;
  }

  /**
   * @type {number}
   */
  get period() {
    return this._period;
  }

  get handle() {
    return this._handle;
  }

  /**
   * Check if the timer is ready.
   * @return {boolean} Return true if timer is ready, otherwise return false.
   */
  isReady() {
    return rclnodejs.isTimerReady(this._handle);
  }

  /**
   * Check if the timer is canceled.
   * @return {boolean} Return true if timer is canceled, otherwise return false.
   */
  isCanceled() {
    return rclnodejs.isTimerCanceled(this._handle);
  }

  /**
   * Cancel the timer.
   * @return {undefined}
   */
  cancel() {
    rclnodejs.cancelTimer(this._handle);
  }

  /**
   * Reset the timer.
   * @return {undefined}
   */
  reset() {
    rclnodejs.resetTimer(this._handle);
  }

  /**
   * Get the interval since the last call of this timer.
   * @return {number} - the interval value - ms.
   */
  timeSinceLastCall() {
    return parseInt(rclnodejs.timerGetTimeSinceLastCall(this._handle), 10);
  }

  /**
   * Get the interval until the next call will happen.
   * @return {number} - the interval value - ms.
   */
  timeUntilNextCall() {
    return parseInt(rclnodejs.timerGetTimeUntilNextCall(this._handle), 10);
  }
}

module.exports = Timer;
