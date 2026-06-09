// Copyright (c) 2026, The Robot Web Tools Contributors
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

import Logging from './logging.js';

const LOGGING_SEVERITY_UNSET = 0;

/**
 * Implements the ROS 2 logging service interfaces for a node.
 *
 * The interfaces implemented are:
 *  rcl_interfaces/srv/GetLoggerLevels
 *  rcl_interfaces/srv/SetLoggerLevels
 *
 * @class
 */
class LoggingService {
  /**
   * Create a new instance.
   * @param {Node} node - The node these services support.
   */
  constructor(node) {
    this._node = node;
    this._isRunning = false;
  }

  /**
   * Get the node this service supports.
   * @return {Node} - The supported node.
   */
  get node() {
    return this._node;
  }

  /**
   * Check if logging services are configured and accepting requests.
   * @return {boolean} - True if services are active; false otherwise.
   */
  isStarted() {
    return this._isRunning;
  }

  /**
   * Configure logging services and begin processing client requests.
   * @return {undefined}
   */
  start() {
    if (this._isRunning) return;

    this._isRunning = true;
    const nodeName = this.node.name();

    this.node.createService(
      'rcl_interfaces/srv/GetLoggerLevels',
      nodeName + '/get_logger_levels',
      (request, response) => this._handleGetLoggerLevels(request, response)
    );

    this.node.createService(
      'rcl_interfaces/srv/SetLoggerLevels',
      nodeName + '/set_logger_levels',
      (request, response) => this._handleSetLoggerLevels(request, response)
    );
  }

  _handleGetLoggerLevels(request, response) {
    const msg = response.template;

    for (const name of request.names) {
      try {
        msg.levels.push({
          name,
          level: Logging.getLogger(name).loggerEffectiveLevel,
        });
      } catch {
        msg.levels.push({
          name,
          level: LOGGING_SEVERITY_UNSET,
        });
      }
    }

    response.send(msg);
  }

  _handleSetLoggerLevels(request, response) {
    const msg = response.template;

    for (const loggerLevel of request.levels) {
      const result = {
        successful: false,
        reason: '',
      };

      try {
        Logging.getLogger(loggerLevel.name).setLoggerLevel(loggerLevel.level);
        result.successful = true;
      } catch (error) {
        result.reason = error.message;
      }

      msg.results.push(result);
    }

    response.send(msg);
  }
}

export default LoggingService;
