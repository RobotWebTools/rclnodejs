declare module 'rclnodejs' {
  /**
   * Encapsulates the lifecycle of an rcl environment from init to shutdown.
   * A Context serves as a container for a ROS2 RCL environment that holds
   * nodes and the resources created by the nodes, e.g.,
   * publishers, subscriptions, actions, services...v
   *
   * @remarks
   * A context has 3 states:
   * ```
   * new Context() --> uninitialized -->
   *                                    |
   *         ---------------------------
   *        |
   *        v
   * rcl.init(context) --> initialized ->
   *                                     |
   *         ----------------------------
   *        |
   *        v
   * rcl.shutdown(context)
   *        or
   * context.shutdown() ---> shutdown
   * ```
   * Must call rclnodejs.init(context) to initialize the context
   * to the usable 'initialized' (valid) state be using.
   *
   * Context objects should not be reused, and are finalized in their destructor.
   */
  class Context {
    /**
     * Access the list of usable (initialized/valid) contexts.
     * @returns Array of valid contexts
     */
    static get instances(): Context[];

    /**
     * Create a new instance in uninitialized state.
     * Call rcl.init(context) to initialize this context state for
     * use in creating nodes, etc.
     */
    constructor();

    /**
     * Test if this context has not been initialized by rcl.init(context).
     * @returns True if context has been initialized; otherwise false
     */
    isUninitialized(): boolean;

    /**
     * Test if this context has been initialized, i.e., rcl.init(context),
     * and not shutdown.
     * @returns True if context has been initialized; otherwise false
     */
    isInitialized(): boolean;

    /**
     * Test if this context has been shutdown, i.e., context.shutdown().
     * @returns True if context has been shutdown; otherwise false
     */
    isShutdown(): boolean;

    /**
     * Access nodes managed by this context.
     * @returns The nodes.
     */
    get nodes(): Node[];

    /**
     * Test if this context is the default one.
     * @returns True if this is the default context; otherwise false.
     */
    isDefaultContext(): boolean;

    /**
     * Shutdown the context, includes destroying all nodes.
     */
    shutdown(): void;

    /**
     * Try to shutdown the context.
     */
    tryShutdown(): void;

    /**
     * Get the global default Context object.
     * @returns The default Context
     */
    static defaultContext(): Context;
  }
}
