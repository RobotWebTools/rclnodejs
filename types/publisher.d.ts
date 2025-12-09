declare module 'rclnodejs' {
  /**
   * Options for publishing a message
   */
  interface PublishOptions {
    /** Override validateMessages setting for this publish call */
    validate?: boolean;
  }

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
     * Whether messages will be validated before publishing.
     */
    willValidateMessage: boolean;

    /**
     * Publish a message
     *
     * @param message - The message to be sent.
     * @param options - Publish options (e.g., { validate: true })
     * @throws MessageValidationError if validation is enabled and message is invalid
     */
    publish(message: MessageType<T> | Buffer, options?: PublishOptions): void;

    /**
     * Set validation options for this publisher.
     * @param options - Validation options
     */
    setValidation(options: MessageValidationOptions): void;

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
     * Manually assert that this Publisher is alive (for RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC).
     */
    assertLiveliness(): void;

    /**
     * Get the logger name for this publisher.
     */
    readonly loggerName: string;
  }
}
