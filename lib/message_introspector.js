// Copyright (c) 2025 Mahmoud Alghalayini. All rights reserved.
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

const loader = require('./interface_loader.js');
const { toPlainObject } = require('../rosidl_gen/message_translator.js');
const { TypeValidationError } = require('./errors.js');

/**
 * A utility class for inspecting ROS 2 message structure without using loader.loadInterface directly.
 * Provides access to message schema, field names, and default values.
 */
class MessageIntrospector {
  #typeClass;
  #typeName;
  #defaultsCache;

  /**
   * Create a new MessageIntrospector for a ROS 2 message type.
   * @param {string} typeName - The full message type name (e.g., 'geometry_msgs/msg/Twist')
   * @throws {TypeValidationError} If typeName is not a non-empty string
   * @throws {Error} If the message type cannot be loaded
   */
  constructor(typeName) {
    if (!typeName || typeof typeName !== 'string') {
      throw new TypeValidationError('typeName', typeName, 'non-empty string', {
        entityType: 'MessageIntrospector',
      });
    }
    this.#typeName = typeName;
    this.#typeClass = loader.loadInterface(typeName);
    this.#defaultsCache = null;
  }

  /**
   * Get the full message type name.
   * @returns {string} The message type name (e.g., 'geometry_msgs/msg/Twist')
   */
  get typeName() {
    return this.#typeName;
  }

  /**
   * Get the underlying ROS message class.
   * @returns {Function} The message type class constructor
   */
  get typeClass() {
    return this.#typeClass;
  }

  /**
   * Get the field names of the message.
   * @returns {string[]} Array of field names
   */
  get fields() {
    const def = this.#typeClass.ROSMessageDef;
    return def.fields.map((f) => f.name);
  }

  /**
   * Get the ROSMessageDef schema for the message type.
   * @returns {object} The message definition schema
   */
  get schema() {
    return this.#typeClass.ROSMessageDef;
  }

  /**
   * Get the default values for all fields.
   * Creates a new instance of the message and converts it to a plain object.
   * Result is cached for performance.
   * @returns {object} A plain object with all default values
   */
  get defaults() {
    if (this.#defaultsCache === null) {
      const instance = new this.#typeClass();
      this.#defaultsCache = toPlainObject(instance);
    }
    return this.#deepClone(this.#defaultsCache);
  }

  /**
   * Deep clone an object.
   * @param {any} obj - Object to clone
   * @returns {any} Cloned object
   * @private
   */
  #deepClone(obj) {
    if (obj === null || typeof obj !== 'object') {
      return obj;
    }

    if (Array.isArray(obj)) {
      return obj.map((item) => this.#deepClone(item));
    }

    if (ArrayBuffer.isView(obj) && !(obj instanceof DataView)) {
      return obj.slice();
    }

    const cloned = {};
    for (const key in obj) {
      if (Object.prototype.hasOwnProperty.call(obj, key)) {
        cloned[key] = this.#deepClone(obj[key]);
      }
    }
    return cloned;
  }
}

module.exports = MessageIntrospector;
