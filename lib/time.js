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
const Duration = require('./duration.js');
const Clock = require('./clock.js');
const Entity = require('./entity.js');

/**
 * @class - Class representing a Time in ROS
 */

class Time {
  /**
   * Create a Time.
   * @param {number} [seconds=0] - The second part of the time.
   * @param {number} [nanoseconds=0] - The nanosecond part of the time.
   * @param {ClockType} [clockType=Clock.ClockType.SYSTEM_TIME] - The clock type.
   */
  constructor(seconds = 0, nanoseconds = 0, clockType = Clock.ClockType.SYSTEM_TIME) {
    if (typeof seconds !== 'number' || typeof nanoseconds !== 'number' ||
        typeof clockType !== 'number') {
      throw new TypeError('Invalid argument');
    }

    if (seconds < 0) {
      throw new RangeError('seconds value must not be negative');
    }

    if (nanoseconds < 0) {
      throw new RangeError('nanoseconds value must not be negative');
    }

    this._nanoseconds = nanoseconds + seconds * 1e9;
    if (Number.isSafeInteger(this._nanoseconds)) {
      this._handle = rclnodejs.createTimePoint(this._nanoseconds, clockType);
      this._clockType = clockType;
    } else {
      throw new RangeError('Total nanoseconds value is too large to be represented in JavaScript');
    }
  }

  /**
   * Get the the clock type of the Time object.
   * @return {ClockType} - The clock type.
   */
  get clockType() {
    return this._clockType;
  }

  /**
   * Get the time in nanosecond.
   * @return {number} - value in nanosecond.
   */
  get nanoseconds() {
    return rclnodejs.getNanoseconds(this._handle);
  }

  /**
   * Get the time as separate seconds and nanoseconds component.
   * @return {object} - object with properties seconds and nanoseconds.
   */
  get secondsAndNanoseconds() {
    let seconds = this.nanoseconds / 1e9;
    let nanoseconds = this.nanoseconds % 1e9;
    return {seconds, nanoseconds};
  }

  /**
   * Add a duration to this time object.
   * @param {Duration} other - The Duration object to be added.
   * @return {Time} Return the result of a new Time object.
   */
  add(other) {
    if (other instanceof Duration) {
      return new Time(0, this._nanoseconds + other.nanoseconds, this._clockType);
    }
    throw new TypeError('Invalid argument');
  }

  /**
   * Subtract a duration/time to this time object.
   * @param {Duration|Time} other - The time to be subtracted.
   * @return {Duration|Time} Return the result.
   */
  sub(other) {
    if (other instanceof Time) {
      if (other._clockType !== this._clockType) {
        throw new TypeError('Can\'t subtract times with different clock types');
      }
      return new Duration(0, this._nanoseconds - other.nanoseconds);
    } else if (other instanceof Duration) {
      return new Time(0, this._nanoseconds - other.nanoseconds, this._clockType);
    }
    throw new TypeError('Invalid argument');
  }

  /**
   * Determine whether two Time objects are equal.
   * @param {Time} other - The time object to be compared.
   * @return {boolean} Return true if they are equal.
   */
  isEqual(other) {
    if (other instanceof Time) {
      if (other._clockType !== this._clockType) {
        throw new TypeError('Can\'t compare times with different clock types');
      }
      return this.nanoseconds === other.nanoseconds;
    }
    throw new TypeError('Invalid argument');
  }

  /**
   * Determine whether two Time objects are not equal.
   * @param {Time} other - The time object to be compared.
   * @return {boolean} Return true if they are not equal.
   */
  isNotEqual(other) {
    return !this.isEqual(other);
  }

  /**
   * Determine whether the time is less than another one.
   * @param {Time} other - The time object to be compared.
   * @return {boolean} Return true if it's less than other.
   */
  isLessThan(other) {
    if (other instanceof Time) {
      if (other._clockType !== this._clockType) {
        throw new TypeError('Can\'t compare times with different clock types');
      }
      return this.nanoseconds < other.nanoseconds;
    }
    throw new TypeError('Invalid argument');
  }

  /**
   * Determine whether the time is less than or equal with another one.
   * @param {Time} other - The time object to be compared.
   * @return {boolean} Return true if it's less than or equal with other.
   */
  isLessThanOrEqual(other) {
    return this.isEqual(other) || this.isLessThan(other);
  }

  /**
   * Determine whether the time is greater than another one.
   * @param {Time} other - The time object to be compared.
   * @return {boolean} Return true if it's greater than other.
   */
  isGreaterThan(other) {
    if (other instanceof Time) {
      if (other._clockType !== this._clockType) {
        throw new TypeError('Can\'t compare times with different clock types');
      }
      return this.nanoseconds > other.nanoseconds;
    }
    throw new TypeError('Invalid argument');
  }

  /**
   * Determine whether the time is greater than or equal with another one.
   * @param {Time} other - The time object to be compared.
   * @return {boolean} Return true if it's greater than or equal with other.
   */
  isGreaterThanOrEqual(other) {
    return this.isEqual(other) || this.isGreaterThan(other);
  }
}

module.exports = Time;
