declare module 'rclnodejs' {
  /**
   * A ROS Publisher that publishes messages on a topic.
   */
  class Publisher extends Entity {
    /**
     * Topic on which messages are published.
     */
    readonly topic: string;

    /**
     * Publish a message
     *
     * @param message - The message to be sent.
     */
    publish(message: Message): void;
  }
}
