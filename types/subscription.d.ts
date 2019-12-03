
declare module 'rclnodejs' {

  /**
   * A ROS Subscription listening for published messages for a specific topic.
   */
  class Subscription extends Entity {

    /**
     * Topic to listen for messages on.
     */
    readonly topic: string;
  }

}
