declare module 'rclnodejs' {
  /**
   * A callback for receiving published messages.
   *
   * @param message - The published message.
   *
   * @remarks
   * See {@link Node#createSubscription | Node.createSubscription}
   * See {@link Node#createPublisher | Node.createPublisher}
   * See {@link Publisher}
   * See {@link Subscription}
   */
  type SubscriptionCallback<T extends TypeClass<MessageTypeClassName>> =
    // * @param message - The published message
    (message: MessageType<T> | Buffer) => void;

  /**
   * A ROS Subscription for published messages on a topic.
   */
  interface Subscription extends Entity {
    /**
     * Topic to listen for messages on.
     */
    readonly topic: string;
  }
}
