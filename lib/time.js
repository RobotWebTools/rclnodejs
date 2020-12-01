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
const ClockType = require('./clock_type.js');
const int64 = require('int64-napi');

/**
 * @class - Class representing a Time in ROS
 */

class Time {
  /**
   * Create a Time.
   * @param {number|string} [seconds=0] - The second part of the time.
   * @param {number|string} [nanoseconds=0] - The nanosecond part of the time.
   * @param {ClockType} [clockType=Clock.ClockType.SYSTEM_TIME] - The clock type.
   */
  constructor(seconds = 0, nanoseconds = 0, clockType = ClockType.SYSTEM_TIME) {
    if (typeof seconds !== 'number' && typeof seconds !== 'string') {
      throw new TypeError('Invalid argument of seconds');
    }

    if (typeof nanoseconds !== 'number' && typeof nanoseconds !== 'string') {
      throw new TypeError('Invalid argument of nanoseconds');
    }

    if (typeof clockType !== 'number') {
      throw new TypeError('Invalid argument of clockType');
    }

    if (
      int64.lt(seconds, 0) ||
      (typeof seconds === 'string' && seconds.startsWith('-'))
    ) {
      throw new RangeError('seconds value must not be negative');
    }

    if (
      int64.lt(nanoseconds, 0) ||
      (typeof nanoseconds === 'string' && nanoseconds.startsWith('-'))
    ) {
      throw new RangeError('nanoseconds value must not be negative');
    }

    this._nanoseconds = int64.from(seconds).multiply(1e9).add(nanoseconds);
    this._handle = rclnodejs.createTimePoint(
      this._nanoseconds.toString(),
      clockType
    );
    this._clockType = clockType;
  }

  /**
   * Get the the clock type of the Time object.
   * @name Time#get:clockType
   * @function
   * @return {ClockType} - The clock type.
   */

  get clockType() {
    return this._clockType;
  }

  /**
   * Get the nanosecond part of the time.
   * @name Time#get:nanoseconds
   * @function
   * @return {number|string} - value in nanosecond, if the value is greater than Number.MAX_SAFE_INTEGER (2^53-1), will be presented in string of decimal format.
   */

  get nanoseconds() {
    let str = rclnodejs.getNanoseconds(this._handle);
    let nano;

    if (str.startsWith('-')) {
      nano = int64.negative(int64.from(str));
    } else {
      nano = int64.from(str);
    }
    if (Number.isFinite(nano.toNumber())) {
      return nano.toNumber();
    }
    return nano.toString();
  }

  /**
   * Get the time as separate seconds and nanoseconds component.
   * @name Time#get:secondsAndNanoseconds
   * @function
   * @return {object} - object with properties seconds and nanoseconds.
   */

  get secondsAndNanoseconds() {
    const seconds = int64.from(this._nanoseconds).divide(1e9).toNumber();
    const nanoseconds = int64.from(this._nanoseconds).mod(1e9).toNumber();
    return { seconds, nanoseconds };
  }

  /**
   * Add a duration to this time object.
   * @param {Duration} other - The Duration object to be added.
   * @return {Time} Return the result of a new Time object.
   */
  add(other) {
    if (other instanceof Duration) {
      return new Time(
        0,
        int64.add(this._nanoseconds, other.nanoseconds).toString(),
        this._clockType
      );
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
        throw new TypeError("Can't subtract times with different clock types");
      }
      return new Duration(
        0,
        int64.subtract(this._nanoseconds, other._nanoseconds).toString()
      );
    } else if (other instanceof Duration) {
      return new Time(
        0,
        int64.subtract(this._nanoseconds, other._nanoseconds).toString(),
        this._clockType
      );
    }
    throw new TypeError('Invalid argument');
  }

  /**
   * Determine whether two Time objects are equal.
   * @param {Time} other - The time object to be compared.
   * @return {boolean} Return true if they are equal.
   */
  eq(other) {
    if (other instanceof Time) {
      if (other._clockType !== this._clockType) {
        throw new TypeError("Can't compare times with different clock types");
      }
      return this._nanoseconds.eq(other.nanoseconds);
    }
    throw new TypeError('Invalid argument');
  }

  /**
   * Determine whether two Time objects are not equal.
   * @param {Time} other - The time object to be compared.
   * @return {boolean} Return true if they are not equal.
   */
  ne(other) {
    if (other instanceof Time) {
      if (other._clockType !== this._clockType) {
        throw new TypeError("Can't compare times with different clock types");
      }
      return this._nanoseconds.ne(other.nanoseconds);
    }
  }

  /**
   * Determine whether the time is less than another one.
   * @param {Time} other - The time object to be compared.
   * @return {boolean} Return true if it's less than other.
   */
  lt(other) {
    if (other instanceof Time) {
      if (other._clockType !== this._clockType) {
        throw new TypeError("Can't compare times with different clock types");
      }
      return this._nanoseconds.lt(other.nanoseconds);
    }
    throw new TypeError('Invalid argument');
  }

  /**
   * Determine whether the time is less than or equal with another one.
   * @param {Time} other - The time object to be compared.
   * @return {boolean} Return true if it's less than or equal with other.
   */
  lte(other) {
    if (other instanceof Time) {
      if (other._clockType !== this._clockType) {
        throw new TypeError("Can't compare times with different clock types");
      }
      return this._nanoseconds.lte(other.nanoseconds);
    }
    throw new TypeError('Invalid argument');
  }

  /**
   * Determine whether the time is greater than another one.
   * @param {Time} other - The time object to be compared.
   * @return {boolean} Return true if it's greater than other.
   */
  gt(other) {
    if (other instanceof Time) {
      if (other._clockType !== this._clockType) {
        throw new TypeError("Can't compare times with different clock types");
      }
      return this._nanoseconds.gt(other.nanoseconds);
    }
    throw new TypeError('Invalid argument');
  }

  /**
   * Determine whether the time is greater than or equal with another one.
   * @param {Time} other - The time object to be compared.
   * @return {boolean} Return true if it's greater than or equal with other.
   */
  gte(other) {
    if (other instanceof Time) {
      if (other._clockType !== this._clockType) {
        throw new TypeError("Can't compare times with different clock types");
      }
      return this._nanoseconds.gte(other.nanoseconds);
    }
    throw new TypeError('Invalid argument');
  }

  /**
   * Create a builtin_interfaces.msg.Time message
   *
   * @return {builtin_interfaces.msg.Time} - The new Time message.
   */
  toMsg() {
    const secondsAndNanoseconds = this.secondsAndNanoseconds;
    return {
      sec: secondsAndNanoseconds.seconds,
      nanosec: secondsAndNanoseconds.nanoseconds,
    };
  }

  /**
   * Create a Time object from a message of builtin_interfaces/msg/Time
   * @param {object} msg - The builtin_interfaces.msg.Time message to be
   *  created from.
   * @param {ClockType} [clockType=Clock.ClockType.ROS_TIME] - The type of the time object.
   * @return {Time} Return the created Time object.
   */
  static fromMsg(msg, clockType = ClockType.ROS_TIME) {
    return new Time(msg.sec, msg.nanosec, clockType);
  }
}

module.exports = Time;
