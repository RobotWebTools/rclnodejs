declare module 'rclnodejs' {
  /**
   * A ROS Subscription for published messages on a topic.
   */
  class Subscription extends Entity {
    /**
     * Topic to listen for messages on.
     */
    readonly topic: string;
  }
}
