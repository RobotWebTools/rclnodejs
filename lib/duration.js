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

/**
 * @class - Class representing a Duration in ROS
 */

class Duration {
  /**
   * Create a Duration.
   * @param {number} [seconds=0] - The second part of the duration.
   * @param {number} [nanoseconds=0] - The nanosecond part of the duration.
   */
  constructor(seconds = 0, nanoseconds = 0) {
    this._nanoseconds = nanoseconds + seconds * 1e9;

    if (!Number.isSafeInteger(this._nanoseconds)) {
      throw new RangeError('Total nanoseconds value is too large to be represented in JavaScript');
    }
  }

  get nanoseconds() {
    return this._nanoseconds;
  }

  /**
   * Determine whether two Duration objects are equal.
   * @param {Duration} other - The Duration object to be compared.
   * @return {boolean} Return true if they are equal.
   */
  isEqual(other) {
    if (other instanceof Duration) {
      return other.nanoseconds === this.nanoseconds;
    }
    throw new TypeError(`Can't compare duration with object of type: ${other.constructor.name}`);
  }

  /**
   * Determine whether two Duration objects are not equal.
   * @param {Duration} other - The Duration object to be compared.
   * @return {boolean} Return true if they are not equal.
   */
  isNotEqual(other) {
    return !this.isEqual(other);
  }

  /**
   * Determine whether the Duration object is less than another one.
   * @param {Duration} other - The Duration object to be compared.
   * @return {boolean} Return true if it's less than other.
   */
  isLessThan(other) {
    if (other instanceof Duration) {
      return this.nanoseconds < other.nanoseconds;
    }
    throw new TypeError('Invalid argument');
  }

  /**
   * Determine whether the Duration object is less than or equal with another one.
   * @param {Duration} other - The Duration object to be compared.
   * @return {boolean} Return true if it's less than or equal with other.
   */
  isLessThanOrEqual(other) {
    return this.isEqual(other) || this.isLessThan(other);
  }

  /**
   * Determine whether the Duration object is greater than another one.
   * @param {Duration} other - The Duration object to be compared.
   * @return {boolean} Return true if it's greater than other.
   */
  isGreaterThan(other) {
    if (other instanceof Duration) {
      return this.nanoseconds > other.nanoseconds;
    }
    throw new TypeError('Invalid argument');
  }

  /**
   * Determine whether the Duration object is greater than or equal with another one.
   * @param {Duration} other - The Duration object to be compared.
   * @return {boolean} Return true if it's greater than or equal with other.
   */
  isGreaterThanOrEqual(other) {
    return this.isEqual(other) || this.isGreaterThan(other);
  }
}

module.exports = Duration;
