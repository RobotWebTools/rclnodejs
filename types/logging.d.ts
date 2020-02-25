declare module 'rclnodejs' {
  // added to match rcl export in index.js.
  // todo: discuss with rclnodejs team about inconsistent naming;
  const logging: Logging;

  /**
   * A ROS Logger.
   */
  class Logging {
    /**
     * Set the logging severity level.
     * Filters log messages with severity less than
     * level.
     *
     * @param level - The logging severity level.
     */
    setLoggerLevel(level: Logging.LoggingSeverity): void;

    /**
     * Get the logging severity level.
     *
     * @returns The severity level of the logger.
     */
    readonly loggerEffectiveLevel: Logging.LoggingSeverity;

    /**
     * Log a message with the DEBUG severity.
     *
     * @param message - message to be logged.
     * @returns True if the message has been logged.
     */
    debug(message: string): boolean;

    /**
     * Log a message with the INFO severity.
     *
     * @param message - message to be logged.
     * @returns True if the message has been logged.
     */
    info(message: string): boolean;

    /**
     * Log a message with the WARN severity.
     *
     * @param message - message to be logged.
     * @returns True if the message has been logged.
     */
    warn(message: string): boolean;

    /**
     * Log a message with the ERROR severity.
     *
     * @param message - message to be logged.
     * @returns True if the message has been logged.
     */
    error(message: string): boolean;

    /**
     * Log a message with the FATAL severity.
     *
     * @param message - message to be logged.
     * @returns True if the message has been logged.
     */
    fatal(message: string): boolean;

    /**
     * Get the logger's LoggingSeverity level.
     *
     * @returns The logging severity level.
     */
    readonly LoggingSeverity: Logging.LoggingSeverity;

    /**
     * Get name of the logger.
     *
     * @returns Logger name.
     */
    readonly name: string;

    /**
     * Create a logger by name.
     *
     * @param  name - name of the logger.
     * @returns New logger.
     */
    static getLogger(name: string): Logging;
  }

  namespace Logging {
    /**
     * LoggingSeverity levels.
     */
    enum LoggingSeverity {
      UNSET = 0,
      DEBUG = 10,
      INFO = 20,
      WARN = 30,
      ERROR = 40,
      FATAL = 50,
    }
  }
}
