
declare module "rclnodejs" {

  // added to match rcl export in index.js. 
  // todo: discuss with rclnodejs team about inconsistent naming;
  const logging: Logging;

  /**
   * @class - Class representing logger in ROS
   * @hideconstructor
   */

  class Logging {

    /**
     * Set the logging severity level.
     * @param {LoggingSeverity} level - The logging severity level.
     * @function
     * @return {undefined}
     */
    setLoggerLevel(level: Logging.LoggingSeverity): void;

    /**
     * Get the logging severity level.
     * @function
     * @return {LoggingSeverity} - The severity level of the logger.
     */
    readonly loggerEffectiveLevel: Logging.LoggingSeverity;

    /**
     * Log a message with the DEBUG severity.
     * @param {string} message - message to be logged.
     * @function
     * @return {bool} Return true if the message has been logged.
     */
    debug(message: string): boolean;

    /**
     * Log a message with the INFO severity.
     * @param {string} message - message to be logged.
     * @function
     * @return {bool} Return true if the message has been logged.
     */
    info(message: string): boolean;

    /**
     * Log a message with the WARN severity.
     * @param {string} message - message to be logged.
     * @function
     * @return {bool} Return true if the message has been logged.
     */
    warn(message: string): boolean;

    /**
     * Log a message with the ERROR severity.
     * @param {string} message - message to be logged.
     * @function
     * @return {bool} Return true if the message has been logged.
     */
    error(message: string): boolean;

    /**
     * Log a message with the FATAL severity.
     * @param {string} message - message to be logged.
     * @function
     * @return {bool} Return true if the message has been logged.
     */
    fatal(message: string): boolean;

    /**
     * Get LoggingSeverity enum.
     * @function
     * @return {LoggingSeverity} Return LoggingSeverity enum.
     */
    readonly LoggingSeverity: Logging.LoggingSeverity;

    /**
     * Get name of the logger.
     * @function
     * @return {string} logger's name.
     */
    readonly name: string;

    /**
     * Create a logger by name.
     * @param {string} name - name of the logger.
     * @function
     * @return {Logging} Return the logger object.
     */
    static getLogger(name: string): Logging
  }

  namespace Logging {

    /**
     * Enum for LoggingSeverity
     * @readonly
     * @enum {number}
     */
    enum LoggingSeverity {
      /** @member {number} */
      UNSET = 0,
      /** @member {number} */
      DEBUG = 10,
      /** @member {number} */
      INFO = 20,
      /** @member {number} */
      WARN = 30,
      /** @member {number} */
      ERROR = 40,
      /** @member {number} */
      FATAL = 50
    }
  }

}
