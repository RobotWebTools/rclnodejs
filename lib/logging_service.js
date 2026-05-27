'use strict';

const Logging = require('./logging.js');

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

module.exports = LoggingService;
