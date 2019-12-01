
declare module "rclnodejs" {

  /**
   * @class - Class representing a TimeSource in ROS
   */
  class TimeSource {
    /**
     * Create a TimeSource.
     * @param {Node} node - The node to be attached.
     */
    constructor(node: Node);

    /**
     * Status that whether the ROS time is active. true when the time is active, otherwise return false.
     * @name TimeSource#get:isRosTimeActive
     * @function
     */
    isRosTimeActive: boolean;

    /**
     * Attach the clock to a Node object.
     * @param {Node} node - The node to be attached.
     * @return {undefined}
     */
    attachNode(node: Node): void;

    /**
     * Detach the node which the clock have attached.
     * @return {undefined}
     */
    detachNode(): void;

    /**
     * Attach the clock to a TimeSource object.
     * @param {Clock} clock - The node to be attached.
     * @return {undefined}
     */
    attachClock(clock: Clock): void;
  }

}

