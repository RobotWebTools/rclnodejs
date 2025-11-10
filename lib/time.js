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
const Duration = require('./duration.js');
const ClockType = require('./clock_type.js');
const { TypeValidationError, RangeValidationError } = require('./errors.js');
const S_TO_NS = 10n ** 9n;

/**
 * @class - Class representing a Time in ROS
 */

class Time {
  /**
   * Create a Time.
   * @param {bigint} [seconds=0] - The second part of the time.
   * @param {bigint} [nanoseconds=0] - The nanosecond part of the time.
   * @param {ClockType} [clockType=Clock.ClockType.SYSTEM_TIME] - The clock type.
   */
  constructor(
    seconds = 0n,
    nanoseconds = 0n,
    clockType = ClockType.SYSTEM_TIME
  ) {
    if (typeof seconds !== 'bigint') {
      throw new TypeValidationError('seconds', seconds, 'bigint', {
        entityType: 'time',
      });
    }

    if (typeof nanoseconds !== 'bigint') {
      throw new TypeValidationError('nanoseconds', nanoseconds, 'bigint', {
        entityType: 'time',
      });
    }

    if (typeof clockType !== 'number') {
      throw new TypeValidationError('clockType', clockType, 'number', {
        entityType: 'time',
      });
    }

    if (seconds < 0n) {
      throw new RangeValidationError('seconds', seconds, '>= 0', {
        entityType: 'time',
      });
    }

    if (nanoseconds < 0n) {
      throw new RangeValidationError('nanoseconds', nanoseconds, '>= 0', {
        entityType: 'time',
      });
    }

    const total = seconds * S_TO_NS + nanoseconds;
    if (total >= 2n ** 63n) {
      throw new RangeValidationError(
        'total nanoseconds',
        total,
        '< 2^63 (max C time point)',
        {
          entityType: 'time',
          details: {
            seconds: seconds,
            nanoseconds: nanoseconds,
            total: total,
          },
        }
      );
    }
    this._nanoseconds = total;
    this._handle = rclnodejs.createTimePoint(this._nanoseconds, clockType);
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
   * @return {bigint} - value in nanosecond.
   */

  get nanoseconds() {
    return rclnodejs.getNanoseconds(this._handle);
  }

  /**
   * Get the time as separate seconds and nanoseconds component.
   * @name Time#get:secondsAndNanoseconds
   * @function
   * @return {object} - object with properties seconds and nanoseconds.
   */

  get secondsAndNanoseconds() {
    const nanoseconds = this._nanoseconds;
    return {
      seconds: nanoseconds / S_TO_NS,
      nanoseconds: nanoseconds % S_TO_NS,
    };
  }

  /**
   * Add a duration to this time object.
   * @param {Duration} other - The Duration object to be added.
   * @return {Time} Return the result of a new Time object.
   */
  add(other) {
    if (other instanceof Duration) {
      return new Time(
        0n,
        this._nanoseconds + other.nanoseconds,
        this._clockType
      );
    }
    throw new TypeValidationError('other', other, 'Duration', {
      entityType: 'time',
    });
  }

  /**
   * Subtract a duration/time to this time object.
   * @param {Duration|Time} other - The time to be subtracted.
   * @return {Duration|Time} Return the result.
   */
  sub(other) {
    if (other instanceof Time) {
      if (other._clockType !== this._clockType) {
        throw new TypeValidationError(
          'other',
          other,
          `Time with clock type ${this._clockType}`,
          {
            entityType: 'time',
            details: {
              expectedClockType: this._clockType,
              providedClockType: other._clockType,
            },
          }
        );
      }
      return new Duration(0n, this._nanoseconds - other._nanoseconds);
    } else if (other instanceof Duration) {
      return new Time(
        0n,
        this._nanoseconds - other._nanoseconds,
        this._clockType
      );
    }
    throw new TypeValidationError('other', other, 'Time or Duration', {
      entityType: 'time',
    });
  }

  /**
   * Determine whether two Time objects are equal.
   * @param {Time} other - The time object to be compared.
   * @return {boolean} Return true if they are equal.
   */
  eq(other) {
    if (other instanceof Time) {
      if (other._clockType !== this._clockType) {
        throw new TypeValidationError(
          'other',
          other,
          `Time with clock type ${this._clockType}`,
          {
            entityType: 'time',
            details: {
              expectedClockType: this._clockType,
              providedClockType: other._clockType,
            },
          }
        );
      }
      return this._nanoseconds === other.nanoseconds;
    }
    throw new TypeValidationError('other', other, 'Time', {
      entityType: 'time',
    });
  }

  /**
   * Determine whether two Time objects are not equal.
   * @param {Time} other - The time object to be compared.
   * @return {boolean} Return true if they are not equal.
   */
  ne(other) {
    if (other instanceof Time) {
      if (other._clockType !== this._clockType) {
        throw new TypeValidationError(
          'other',
          other,
          `Time with clock type ${this._clockType}`,
          {
            entityType: 'time',
            details: {
              expectedClockType: this._clockType,
              providedClockType: other._clockType,
            },
          }
        );
      }
      return this._nanoseconds !== other.nanoseconds;
    }
    throw new TypeValidationError('other', other, 'Time', {
      entityType: 'time',
    });
  }

  /**
   * Determine whether the time is less than another one.
   * @param {Time} other - The time object to be compared.
   * @return {boolean} Return true if it's less than other.
   */
  lt(other) {
    if (other instanceof Time) {
      if (other._clockType !== this._clockType) {
        throw new TypeValidationError(
          'other',
          other,
          `Time with clock type ${this._clockType}`,
          {
            entityType: 'time',
            details: {
              expectedClockType: this._clockType,
              providedClockType: other._clockType,
            },
          }
        );
      }
      return this._nanoseconds < other.nanoseconds;
    }
    throw new TypeValidationError('other', other, 'Time', {
      entityType: 'time',
    });
  }

  /**
   * Determine whether the time is less than or equal with another one.
   * @param {Time} other - The time object to be compared.
   * @return {boolean} Return true if it's less than or equal with other.
   */
  lte(other) {
    if (other instanceof Time) {
      if (other._clockType !== this._clockType) {
        throw new TypeValidationError(
          'other',
          other,
          `Time with clock type ${this._clockType}`,
          {
            entityType: 'time',
            details: {
              expectedClockType: this._clockType,
              providedClockType: other._clockType,
            },
          }
        );
      }
      return this._nanoseconds <= other.nanoseconds;
    }
    throw new TypeValidationError('other', other, 'Time', {
      entityType: 'time',
    });
  }

  /**
   * Determine whether the time is greater than another one.
   * @param {Time} other - The time object to be compared.
   * @return {boolean} Return true if it's greater than other.
   */
  gt(other) {
    if (other instanceof Time) {
      if (other._clockType !== this._clockType) {
        throw new TypeValidationError(
          'other',
          other,
          `Time with clock type ${this._clockType}`,
          {
            entityType: 'time',
            details: {
              expectedClockType: this._clockType,
              providedClockType: other._clockType,
            },
          }
        );
      }
      return this._nanoseconds > other.nanoseconds;
    }
    throw new TypeValidationError('other', other, 'Time', {
      entityType: 'time',
    });
  }

  /**
   * Determine whether the time is greater than or equal with another one.
   * @param {Time} other - The time object to be compared.
   * @return {boolean} Return true if it's greater than or equal with other.
   */
  gte(other) {
    if (other instanceof Time) {
      if (other._clockType !== this._clockType) {
        throw new TypeValidationError(
          'other',
          other,
          `Time with clock type ${this._clockType}`,
          {
            entityType: 'time',
            details: {
              expectedClockType: this._clockType,
              providedClockType: other._clockType,
            },
          }
        );
      }
      return this._nanoseconds >= other.nanoseconds;
    }
    throw new TypeValidationError('other', other, 'Time', {
      entityType: 'time',
    });
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
    return new Time(BigInt(msg.sec), BigInt(msg.nanosec), clockType);
  }
}

module.exports = Time;
