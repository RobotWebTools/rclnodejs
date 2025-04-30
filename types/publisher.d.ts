declare module 'rclnodejs' {
  /**
   * A ROS Publisher that publishes messages on a topic.
   */
  interface Publisher<T extends TypeClass<MessageTypeClassName>>
    extends Entity {
    /**
     * Topic on which messages are published.
     */
    readonly topic: string;
    /**
     * Publish a message
     *
     * @param message - The message to be sent.
     */
    publish(message: MessageType<T> | Buffer): void;

    /**
     * Get the number of subscriptions to this publisher.
     * @returns The number of subscriptions
     */
    subscriptionCount(): number;
  }
}
