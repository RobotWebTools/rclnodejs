declare module 'rclnodejs' {
  /**
   *A ROS TimeSource
   */
  class TimeSource {
    /**
     * Create a TimeSource.
     *
     * @param node - The node to be attached.
     */
    constructor(node: Node);

    /**
     * Status of whether the ROS time is active.
     * True when the time is active, otherwise return false.
     */
    isRosTimeActive: boolean;

    /**
     * Attach the clock to a Node object.
     *
     * @param  node - The node to be attached.
     */
    attachNode(node: Node): void;

    /**
     * Detach the node which the clock have attached.
     */
    detachNode(): void;

    /**
     * Attach the clock to a TimeSource object.
     *
     * @param  clock - The node to be attached.
     */
    attachClock(clock: Clock): void;
  }
}
