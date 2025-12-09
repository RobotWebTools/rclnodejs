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
const { TypeValidationError, NameValidationError } = require('./errors.js');

/**
 * An object - Representing a validator in ROS.
 * @exports validator
 */
let validator = {
  _createErrorFromValidation: function (result, nameValue, nameType) {
    return new NameValidationError(nameValue, nameType, result[0], result[1]);
  },

  /**
   * Validate a given topic or service name, and throw an error if invalid.
   * @param {string} topic - The name of topic/service. Must be fully-qualified and already expanded.
   * @returns {true} Always returns true if valid.
   * @throws {TypeValidationError} If topic is not a string.
   * @throws {NameValidationError} If the topic name is invalid.
   */
  validateFullTopicName(topic) {
    if (typeof topic !== 'string') {
      throw new TypeValidationError('topic', topic, 'string');
    }

    let result = rclnodejs.validateFullTopicName(topic);
    if (result === null) {
      return true;
    }
    throw this._createErrorFromValidation(result, topic, 'topic');
  },

  /**
   * Check if a fully-qualified topic name is valid without throwing.
   * @param {string} topic - The name of topic/service. Must be fully-qualified and already expanded.
   * @returns {boolean} True if valid, false otherwise.
   */
  isValidFullTopicName(topic) {
    if (typeof topic !== 'string') {
      return false;
    }
    return rclnodejs.validateFullTopicName(topic) === null;
  },

  /**
   * Validate a given node name, and throw an error if invalid.
   * @param {string} name - The name of node.
   * @returns {true} Always returns true if valid.
   * @throws {TypeValidationError} If name is not a string.
   * @throws {NameValidationError} If the node name is invalid.
   */
  validateNodeName(name) {
    if (typeof name !== 'string') {
      throw new TypeValidationError('name', name, 'string');
    }

    let result = rclnodejs.validateNodeName(name);
    if (result === null) {
      return true;
    }
    throw this._createErrorFromValidation(result, name, 'node');
  },

  /**
   * Check if a node name is valid without throwing.
   * @param {string} name - The name of node.
   * @returns {boolean} True if valid, false otherwise.
   */
  isValidNodeName(name) {
    if (typeof name !== 'string') {
      return false;
    }
    return rclnodejs.validateNodeName(name) === null;
  },

  /**
   * Validate a given topic or service name, and throw an error if invalid.
   * @param {string} topic - The name of topic/service. Does not have to be fully-qualified.
   * @returns {true} Always returns true if valid.
   * @throws {TypeValidationError} If topic is not a string.
   * @throws {NameValidationError} If the topic name is invalid.
   */
  validateTopicName(topic) {
    if (typeof topic !== 'string') {
      throw new TypeValidationError('topic', topic, 'string');
    }

    let result = rclnodejs.validateTopicName(topic);
    if (result === null) {
      return true;
    }
    throw this._createErrorFromValidation(result, topic, 'topic');
  },

  /**
   * Check if a topic name is valid without throwing.
   * @param {string} topic - The name of topic/service. Does not have to be fully-qualified.
   * @returns {boolean} True if valid, false otherwise.
   */
  isValidTopicName(topic) {
    if (typeof topic !== 'string') {
      return false;
    }
    return rclnodejs.validateTopicName(topic) === null;
  },

  /**
   * Validate a given namespace, and throw an error if invalid.
   * @param {string} namespace - The namespace to be validated.
   * @returns {true} Always returns true if valid.
   * @throws {TypeValidationError} If namespace is not a string.
   * @throws {NameValidationError} If the namespace is invalid.
   */
  validateNamespace(namespace) {
    if (typeof namespace !== 'string') {
      throw new TypeValidationError('namespace', namespace, 'string');
    }

    let result = rclnodejs.validateNamespace(namespace);
    if (result === null) {
      return true;
    }
    throw this._createErrorFromValidation(result, namespace, 'namespace');
  },

  /**
   * Check if a namespace is valid without throwing.
   * @param {string} namespace - The namespace to be validated.
   * @returns {boolean} True if valid, false otherwise.
   */
  isValidNamespace(namespace) {
    if (typeof namespace !== 'string') {
      return false;
    }
    return rclnodejs.validateNamespace(namespace) === null;
  },
};

module.exports = validator;
