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
const S_TO_NS = 10n ** 9n;

/**
 * @class - Class representing a Duration in ROS
 */

class Duration {
  /**
   * Create a Duration.
   * @param {bigint} [seconds=0] - The second part of the duration.
   * @param {bigint} [nanoseconds=0] - The nanosecond part of the duration.
   */
  constructor(seconds = 0n, nanoseconds = 0n) {
    if (typeof seconds !== 'bigint') {
      throw new TypeError('Invalid argument of seconds');
    }

    if (typeof nanoseconds !== 'bigint') {
      throw new TypeError('Invalid argument of nanoseconds');
    }

    const total = seconds * S_TO_NS + nanoseconds;
    if (total >= 2n ** 63n) {
      throw new RangeError(
        'Total nanoseconds value is too large to store in C time point.'
      );
    }

    this._nanoseconds = total;
    this._handle = rclnodejs.createDuration(this._nanoseconds);
  }

  /**
   * Get the nanosecond part of the Duration.
   * @name Duration#get:nanoseconds
   * @function
   * @return {bigint} - value in nanosecond.
   */

  get nanoseconds() {
    return rclnodejs.getDurationNanoseconds(this._handle);
  }

  /**
   * Determine whether two Duration objects are equal.
   * @param {Duration} other - The Duration object to be compared.
   * @return {boolean} Return true if they are equal.
   */
  eq(other) {
    if (other instanceof Duration) {
      return this._nanoseconds === other.nanoseconds;
    }
    throw new TypeError(
      `Can't compare duration with object of type: ${other.constructor.name}`
    );
  }

  /**
   * Determine whether two Duration objects are not equal.
   * @param {Duration} other - The Duration object to be compared.
   * @return {boolean} Return true if they are not equal.
   */
  ne(other) {
    if (other instanceof Duration) {
      return this._nanoseconds !== other.nanoseconds;
    }
    throw new TypeError('Invalid argument');
  }

  /**
   * Determine whether the Duration object is less than another one.
   * @param {Duration} other - The Duration object to be compared.
   * @return {boolean} Return true if it's less than other.
   */
  lt(other) {
    if (other instanceof Duration) {
      return this._nanoseconds < other.nanoseconds;
    }
    throw new TypeError('Invalid argument');
  }

  /**
   * Determine whether the Duration object is less than or equal with another one.
   * @param {Duration} other - The Duration object to be compared.
   * @return {boolean} Return true if it's less than or equal with other.
   */
  lte(other) {
    if (other instanceof Duration) {
      return this._nanoseconds <= other.nanoseconds;
    }
    throw new TypeError('Invalid argument');
  }

  /**
   * Determine whether the Duration object is greater than another one.
   * @param {Duration} other - The Duration object to be compared.
   * @return {boolean} Return true if it's greater than other.
   */
  gt(other) {
    if (other instanceof Duration) {
      return this._nanoseconds > other.nanoseconds;
    }
    throw new TypeError('Invalid argument');
  }

  /**
   * Determine whether the Duration object is greater than or equal with another one.
   * @param {Duration} other - The Duration object to be compared.
   * @return {boolean} Return true if it's greater than or equal with other.
   */
  gte(other) {
    if (other instanceof Duration) {
      return this._nanoseconds >= other.nanoseconds;
    }
    throw new TypeError('Invalid argument');
  }
}

module.exports = Duration;
