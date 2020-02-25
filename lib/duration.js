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
const int64 = require('int64-napi');

/**
 * @class - Class representing a Duration in ROS
 */

class Duration {
  /**
   * Create a Duration.
   * @param {number|string} [seconds=0] - The second part of the duration.
   * @param {number|string} [nanoseconds=0] - The nanosecond part of the duration.
   */
  constructor(seconds = 0, nanoseconds = 0) {
    if (typeof seconds !== 'number' && typeof seconds !== 'string') {
      throw new TypeError('Invalid argument of seconds');
    }

    if (typeof nanoseconds !== 'number' && typeof nanoseconds !== 'string') {
      throw new TypeError('Invalid argument of nanoseconds');
    }

    let secondInt64 = int64.from(seconds);
    let nanoInt64 = int64.from(nanoseconds);

    if (typeof seconds === 'string' && seconds.startsWith('-')) {
      secondInt64 = int64.negative(secondInt64);
    }
    if (typeof nanoseconds === 'string' && nanoseconds.startsWith('-')) {
      nanoInt64 = int64.negative(nanoInt64);
    }
    this._nanoseconds = secondInt64.multiply(1e9).add(nanoInt64);
    this._handle = rclnodejs.createDuration(this._nanoseconds.toString());
  }

  /**
   * Get the nanosecond part of the Duration.
   * @name Duration#get:nanoseconds
   * @function
   * @return {number|string} - value in nanosecond, if the value is greater than Number.MAX_SAFE_INTEGER (2^53-1), will be presented in string of decimal format.
   */

  get nanoseconds() {
    let nanoStr = rclnodejs.getDurationNanoseconds(this._handle);
    let nano;

    if (nanoStr.startsWith('-')) {
      nano = int64.negative(int64.from(nanoStr));
    } else {
      nano = int64.from(nanoStr);
    }
    if (Number.isFinite(nano.toNumber())) {
      return nano.toNumber();
    }
    return nano.toString();
  }

  /**
   * Determine whether two Duration objects are equal.
   * @param {Duration} other - The Duration object to be compared.
   * @return {boolean} Return true if they are equal.
   */
  eq(other) {
    if (other instanceof Duration) {
      return this._nanoseconds.eq(other.nanoseconds);
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
      return this._nanoseconds.ne(other.nanoseconds);
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
      return this._nanoseconds.lt(other.nanoseconds);
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
      return this._nanoseconds.lte(other.nanoseconds);
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
      return this._nanoseconds.gt(other.nanoseconds);
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
      return this._nanoseconds.gte(other.nanoseconds);
    }
    throw new TypeError('Invalid argument');
  }
}

module.exports = Duration;
