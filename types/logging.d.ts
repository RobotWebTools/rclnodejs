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
     * Create a child logger.
     *
     * @param name - name of the child logger.
     * @returns The child logger object.
     */
    getChild(name: string): Logging;

    /**
     * Destroy the logger and remove it from the parent logger if it is a child logger.
     */
    destroy(): void;

    /**
     * Create a logger by name.
     *
     * @param  name - name of the logger.
     * @returns New logger.
     */
    static getLogger(name: string): Logging;

    /**
     * Configure the logging system with the given context.
     *
     * @param context - The context to configure logging for.
     */
    static configure(context: Context): void;

    /**
     * Shutdown the logging system.
     */
    static shutdown(): void;

    /**
     * Get the logging directory.
     *
     * @returns The logging directory.
     */
    static getLoggingDirectory(): string;
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
