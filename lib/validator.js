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
 * An object - Representing a validator in ROS.
 * @exports validator
 */
let validator = {
  _createErrorFromValidation: function (result) {
    let err = new Error(result[0]);
    err.invalidIndex = result[1];
    return err;
  },

  /**
   * Validate a given topic or service name, and throw an error if invalid.
   * @param {string} topic - The name of topic/service. and it must be fully-qualified and already expanded.
   * @return {boolean} - True if it is valid.
   */
  validateFullTopicName(topic) {
    if (typeof topic !== 'string') {
      throw new TypeError('Invalid argument');
    }

    let result = rclnodejs.validateFullTopicName(topic);
    if (result === null) {
      return true;
    }
    throw this._createErrorFromValidation(result);
  },

  /**
   * Validate a given node name, and throw an error if invalid.
   * @param {string} name - The name of node.
   * @return {boolean} - True if it is valid.
   */
  validateNodeName(name) {
    if (typeof name !== 'string') {
      throw new TypeError('Invalid argument');
    }

    let result = rclnodejs.validateNodeName(name);
    if (result === null) {
      return true;
    }
    throw this._createErrorFromValidation(result);
  },

  /**
   * Validate a given topic or service name, and throw an error if invalid.
   * @param {string} topic - The name of topic/service and does not have to be fully-qualified and is not expanded.
   * @return {boolean} - True if it is valid.
   */
  validateTopicName(topic) {
    if (typeof topic !== 'string') {
      throw new TypeError('Invalid argument');
    }

    let result = rclnodejs.validateTopicName(topic);
    if (result === null) {
      return true;
    }
    throw this._createErrorFromValidation(result);
  },

  /**
   * Validate a given namespace, and throw an error if invalid.
   * @param {string} namespace - The namespace to be validated
   * @return {boolean} - True if it is valid.
   */
  validateNamespace(namespace) {
    if (typeof namespace !== 'string') {
      throw new TypeError('Invalid argument');
    }

    let result = rclnodejs.validateNamespace(namespace);
    if (result === null) {
      return true;
    }
    throw this._createErrorFromValidation(result);
  },
};

module.exports = validator;
