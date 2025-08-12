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

    /**
     * Wait until all published message data is acknowledged or until the specified timeout elapses
     *
     * If the timeout is negative then this function will block indefinitely until all published
     * message data is acknowledged.
     * If the timeout is 0 then it will check if all published message has been acknowledged without
     * waiting.
     * If the timeout is greater than 0 then it will return after that period of time has elapsed or
     * all published message data is acknowledged.
     *
     * Raises an error if failed, such as the middleware not supporting this feature.
     *
     * @param {timeout} timeout - The duration to wait for all published message data to be acknowledged in nanoseconds.
     * @return {boolean} `true` if all published message data is acknowledged before the timeout, otherwise `false`.
     */
    waitForAllAcked(timeout: bigint): boolean;

    /**
     * Get the logger name for this publisher.
     */
    readonly loggerName: string;
  }
}
