// Copyright (c) 2020 Matt Richard. All rights reserved.
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

/**
 * @class - Wraps a promise allowing it to be resolved elsewhere.
 * @ignore
 */

class Deferred {
  /**
   * Creates a new deferred Promise.
   * @constructor
   */
  constructor() {
    this._promise = new Promise((resolve) => (this._resolve = resolve));
  }

  /**
   * Gets the underlying promise.
   */
  get promise() {
    return this._promise;
  }

  /**
   * Resolves the deferred promise.
   * @param {*} result - The value to resolve the promise with.
   * @returns {undefined}
   */
  setResult(result) {
    if (this._resolve) {
      this._result = result;
      this._resolve(result);
      this._resolve = null;
    }
  }

  /**
   * Sets a function to be called after the promise resolved
   * @param {function} callback - Function to be called.
   * @returns {undefined}
   */
  setDoneCallback(callback) {
    if (typeof callback !== 'function') {
      throw new TypeError('Invalid parameter');
    }

    this._promise.finally(() => callback(this._result));
  }
}

module.exports = Deferred;
