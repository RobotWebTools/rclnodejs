
declare module 'rclnodejs' {

  /**
   * Class representing a Context in ROS
   */
  class Context {

    /**
     * Shutdown the context.
     * @return {undefined}
     */
    shutdown(): void;

    /**
     * Try to shutdown the context.
     * @return {undefined}
     */
    tryShutdown(): void;

    /**
     * Get the global default Context object.
     * @return {Context} - default Context
     */
    static defaultContext(): Context;

    /**
     * Shutdown the default context.
     * @return {undefined}
     */
    static shutdownDefaultContext(): void;
    
  }
}
