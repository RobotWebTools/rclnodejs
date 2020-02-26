declare module 'rclnodejs' {
  /**
   * Encapsulates the lifecycle of the module from init to shutdown.
   *
   * @remarks
   * Context objects should not be reused, and are finalized in their destructor.
   */
  class Context {
    /**
     * Shutdown the context.
     */
    shutdown(): void;

    /**
     * Try to shutdown the context.
     */
    tryShutdown(): void;

    /**
     * Get the global default Context object.
     *
     * @returns The default Context
     */
    static defaultContext(): Context;

    /**
     * Shutdown the default context.
     */
    static shutdownDefaultContext(): void;
  }
}
