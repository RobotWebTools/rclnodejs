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

const rclnodejs = require('./native_loader.js');
const DistroUtils = require('./distro.js');

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
   * @type {bigint} - The period of the timer in nanoseconds.
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
   * @return {bigint} - the interval value in nanoseconds.
   */
  timeSinceLastCall() {
    return rclnodejs.timerGetTimeSinceLastCall(this._handle);
  }

  /**
   * Get the interval until the next call will happen.
   * @return {bigint} - the interval value in nanoseconds.
   */
  timeUntilNextCall() {
    return rclnodejs.timerGetTimeUntilNextCall(this._handle);
  }

  /**
   * Change the timer period.
   * @param {bigint} period - The new period in nanoseconds.
   * @return {undefined}
   */
  changeTimerPeriod(period) {
    rclnodejs.changeTimerPeriod(this._handle, period);
  }

  /**
   * Get the timer period.
   * @return {bigint} - The period in nanoseconds.
   */
  get timerPeriod() {
    return rclnodejs.getTimerPeriod(this._handle);
  }

  /**
   * Call a timer and starts counting again, retrieves actual and expected call time.
   * @return {object} - The timer information.
   */
  callTimerWithInfo() {
    if (DistroUtils.getDistroId() <= DistroUtils.getDistroId('humble')) {
      console.warn(
        'callTimerWithInfo is not supported by this version of ROS 2'
      );
      return;
    }
    return rclnodejs.callTimerWithInfo(this._handle);
  }
}

module.exports = Timer;
