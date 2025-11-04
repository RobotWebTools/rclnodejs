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

const { ValidationError } = require('./errors.js');

/**
 * Check if a value is a TypedArray
 * @param {*} value - The value to check
 * @returns {boolean} True if the value is a TypedArray
 */
function isTypedArray(value) {
  return ArrayBuffer.isView(value) && !(value instanceof DataView);
}

/**
 * Check if a value needs JSON conversion (BigInt, functions, etc.)
 * @param {*} value - The value to check
 * @returns {boolean} True if the value needs special JSON handling
 */
function needsJSONConversion(value) {
  return (
    typeof value === 'bigint' ||
    typeof value === 'function' ||
    typeof value === 'undefined' ||
    value === Infinity ||
    value === -Infinity ||
    (typeof value === 'number' && isNaN(value))
  );
}

/**
 * Convert a message to plain arrays (TypedArray -> regular Array)
 * @param {*} obj - The object to convert
 * @returns {*} The converted object with plain arrays
 */
function toPlainArrays(obj) {
  if (obj === null || obj === undefined) {
    return obj;
  }

  if (isTypedArray(obj)) {
    return Array.from(obj);
  }

  if (Array.isArray(obj)) {
    return obj.map((item) => toPlainArrays(item));
  }

  if (typeof obj === 'object' && obj !== null) {
    const result = {};
    for (const key in obj) {
      if (Object.prototype.hasOwnProperty.call(obj, key)) {
        result[key] = toPlainArrays(obj[key]);
      }
    }
    return result;
  }

  return obj;
}

/**
 * Convert a message to be fully JSON-safe
 * @param {*} obj - The object to convert
 * @returns {*} The JSON-safe converted object
 */
function toJSONSafe(obj) {
  if (obj === null || obj === undefined) {
    return obj;
  }

  if (isTypedArray(obj)) {
    return Array.from(obj).map((item) => toJSONSafe(item));
  }

  if (needsJSONConversion(obj)) {
    if (typeof obj === 'bigint') {
      // Convert BigInt to string with 'n' suffix to indicate it was a BigInt
      return obj.toString() + 'n';
    }
    if (obj === Infinity) return 'Infinity';
    if (obj === -Infinity) return '-Infinity';
    if (typeof obj === 'number' && isNaN(obj)) return 'NaN';
    if (typeof obj === 'undefined') return null;
    if (typeof obj === 'function') return '[Function]';
  }

  if (Array.isArray(obj)) {
    return obj.map((item) => toJSONSafe(item));
  }

  if (typeof obj === 'object' && obj !== null) {
    const result = {};
    for (const key in obj) {
      if (Object.prototype.hasOwnProperty.call(obj, key)) {
        result[key] = toJSONSafe(obj[key]);
      }
    }
    return result;
  }

  return obj;
}

/**
 * Convert a message to a JSON string
 * @param {*} obj - The object to convert
 * @param {number} [space] - Space parameter for JSON.stringify formatting
 * @returns {string} The JSON string representation
 */
function toJSONString(obj, space) {
  const jsonSafeObj = toJSONSafe(obj);
  return JSON.stringify(jsonSafeObj, null, space);
}

/**
 * Apply serialization mode conversion to a message object
 * @param {*} message - The message object to convert
 * @param {string} serializationMode - The serialization mode ('default', 'plain', 'json')
 * @returns {*} The converted message
 */
function applySerializationMode(message, serializationMode) {
  switch (serializationMode) {
    case 'default':
      // No conversion needed - use native rclnodejs behavior
      return message;

    case 'plain':
      // Convert TypedArrays to regular arrays
      return toPlainArrays(message);

    case 'json':
      // Convert to fully JSON-safe format
      return toJSONSafe(message);

    default:
      throw new ValidationError(
        `Invalid serializationMode: ${serializationMode}. Valid modes are: 'default', 'plain', 'json'`,
        {
          code: 'INVALID_SERIALIZATION_MODE',
          argumentName: 'serializationMode',
          providedValue: serializationMode,
          expectedType: "'default' | 'plain' | 'json'",
        }
      );
  }
}

/**
 * Validate serialization mode
 * @param {string} mode - The serialization mode to validate
 * @returns {boolean} True if valid
 */
function isValidSerializationMode(mode) {
  return ['default', 'plain', 'json'].includes(mode);
}

module.exports = {
  isTypedArray,
  needsJSONConversion,
  toPlainArrays,
  toJSONSafe,
  toJSONString,
  applySerializationMode,
  isValidSerializationMode,
};
