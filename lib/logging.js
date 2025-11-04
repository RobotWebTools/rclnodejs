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

const path = require('path');
const rclnodejs = require('./native_loader.js');
const { TypeValidationError } = require('./errors.js');

/**
 * Enum for LoggingSeverity
 * @readonly
 * @enum {number}
 */
let LoggingSeverity = {
  /** @member {number} */
  UNSET: 0,
  /** @member {number} */
  DEBUG: 10,
  /** @member {number} */
  INFO: 20,
  /** @member {number} */
  WARN: 30,
  /** @member {number} */
  ERROR: 40,
  /** @member {number} */
  FATAL: 50,
};

class Caller {
  constructor() {
    this._info = {
      functionName: 'unknown',
      fileName: 'unknown',
      lineNumber: 'unknown',
    };

    const stackLines = new Error().stack.split('\n');

    // Adjust the index (usually 3 or 4) to correctly point to the caller frame.
    const callerFrame = stackLines[4] || stackLines[3];
    // Match both named and anonymous function stack frames.
    const frameRegex = /^\s*at\s+(?:(.+)\s+\()?(.+):(\d+):(\d+)\)?$/;
    const match = callerFrame.match(frameRegex);
    if (match && match.length === 5) {
      this._info.functionName = match[1] || '(anonymous)';
      this._info.fileName = path.basename(match[2]);
      this._info.lineNumber = match[3];
    } else {
      // Handle anonymous functions or different stack formats.
      const altMatch = callerFrame.match(/at\s+(.*):(\d+):(\d+)/i);
      if (altMatch && altMatch.length >= 4) {
        this._info.functionName = '(anonymous)';
        this._info.fileName = path.basename(altMatch[1]);
        this._info.lineNumber = altMatch[2];
      }
    }
  }

  get functionName() {
    return this._info.functionName;
  }

  get lineNumber() {
    return this._info.lineNumber;
  }

  get fileName() {
    return this._info.fileName;
  }
}

/**
 * @class - Class representing logger in ROS
 * @hideconstructor
 */

class Logging {
  constructor(name) {
    this._name = name;
  }

  /**
   * Set the logging severity level.
   * @param {LoggingSeverity} level - The logging severity level.
   * @function
   * @return {undefined}
   */
  setLoggerLevel(level) {
    if (typeof level !== 'number') {
      throw new TypeValidationError('level', level, 'number', {
        entityType: 'logger',
        entityName: this._name,
      });
    }
    rclnodejs.setLoggerLevel(this._name, level);
  }

  /**
   * Get the logging severity level.
   * @function
   * @return {LoggingSeverity} - The severity level of the logger.
   */
  get loggerEffectiveLevel() {
    return rclnodejs.getLoggerEffectiveLevel(this._name);
  }

  /**
   * Log a message with the DEBUG severity.
   * @param {string} message - message to be logged.
   * @function
   * @return {bool} Return true if the message has been logged.
   */
  debug(message) {
    return this._log(message, LoggingSeverity.DEBUG);
  }

  /**
   * Log a message with the INFO severity.
   * @param {string} message - message to be logged.
   * @function
   * @return {bool} Return true if the message has been logged.
   */
  info(message) {
    return this._log(message, LoggingSeverity.INFO);
  }

  /**
   * Log a message with the WARN severity.
   * @param {string} message - message to be logged.
   * @function
   * @return {bool} Return true if the message has been logged.
   */
  warn(message) {
    return this._log(message, LoggingSeverity.WARN);
  }

  /**
   * Log a message with the ERROR severity.
   * @param {string} message - message to be logged.
   * @function
   * @return {bool} Return true if the message has been logged.
   */
  error(message) {
    return this._log(message, LoggingSeverity.ERROR);
  }

  /**
   * Log a message with the FATAL severity.
   * @param {string} message - message to be logged.
   * @function
   * @return {bool} Return true if the message has been logged.
   */
  fatal(message) {
    return this._log(message, LoggingSeverity.FATAL);
  }

  _log(message, severity) {
    if (typeof message !== 'string') {
      throw new TypeValidationError('message', message, 'string', {
        entityType: 'logger',
        entityName: this._name,
      });
    }

    let caller = new Caller();
    return rclnodejs.log(
      this._name,
      severity,
      message,
      caller.functionName,
      parseInt(caller.lineNumber, 10),
      caller.fileName
    );
  }

  /**
   * Get LoggingSeverity enum.
   * @function
   * @return {LoggingSeverity} Return LoggingSeverity enum.
   */
  get LoggingSeverity() {
    return LoggingSeverity;
  }

  /**
   * Get name of the logger.
   * @function
   * @return {string} logger's name.
   */
  get name() {
    return this._name;
  }

  /**
   * Create a logger by name.
   * @param {string} name - name of the logger.
   * @function
   * @return {Logging} Return the logger object.
   */
  static getLogger(name) {
    if (typeof name !== 'string') {
      throw new TypeValidationError('name', name, 'string', {
        entityType: 'logger',
      });
    }
    return new Logging(name);
  }
}

module.exports = Logging;
