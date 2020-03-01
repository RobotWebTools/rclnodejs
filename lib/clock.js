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

const rclnodejs = require('bindings')('rclnodejs');
const Time = require('./time.js');
const ClockType = require('./clock_type.js');

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
   * Return the current time.
   * @return {Time} Return the current time.
   */
  now() {
    let time = rclnodejs.clockGetNow(this._handle);
    return new Time(time.sec, time.nanosec, this._clockType);
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
      throw new TypeError('Invalid argument, must be type of Time');
    }
    rclnodejs.setRosTimeOverride(this._handle, time._handle);
  }
}

Clock.ClockType = ClockType;

module.exports = { Clock, ROSClock };
