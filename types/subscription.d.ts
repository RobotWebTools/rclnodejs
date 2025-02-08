declare module 'rclnodejs' {
  /**
   * A callback for receiving published messages.
   *
   * @param message - The published message.
   *
   * @remarks
   * See {@link Node#createSubscription | Node.createSubscription}
   * See {@link SubscriptionContentFilter}
   * See {@link Node#createPublisher | Node.createPublisher}
   * See {@link Publisher}
   * See {@link Subscription}
   */
  type SubscriptionCallback<T extends TypeClass<MessageTypeClassName>> =
    // * @param message - The published message
    (message: MessageType<T>) => void;

  /**
   * A callback for receiving published raw messages.
   *
   * @param message - The published message.
   *
   * @remarks
   * See {@link Node#createSubscription | Node.createSubscription}
   * See {@link SubscriptionContentFilter}
   * See {@link Node#createPublisher | Node.createPublisher}
   * See {@link Publisher}
   * See {@link Subscription}
   */
  type SubscriptionWithRawMessageCallback =
    // * @param message - The published raw message
    (message: Buffer) => void;

  /**
   * A ROS Subscription for published messages on a topic.
   */
  interface Subscription extends Entity {
    /**
     * Topic to listen for messages on.
     */
    readonly topic: string;

    /**
     * Specifies if messages are in raw (binary) format
     */
    readonly isRaw: boolean;

    /**
     * Test if the RMW supports content-filtered topics and that this subscription
     * is configured with a well formed content-filter.
     * @returns {boolean} True if content-filtering will be applied; otherwise false.
     */
    hasContentFilter(): boolean;

    /**
     * Set a content-filter if the RMW supports content-filtered topics.
     * @param contentFilter - The content-filter description to apply.
     * @returns True if successful; false otherwise
     * @remarks
     * @see {@link https://www.omg.org/spec/DDS/1.4/PDF|DDS 1.4 specification, Annex B}
     */
    setContentFilter(filter: SubscriptionContentFilter): boolean;

    /**
     * Clear the current content-filter. No filtering is to be applied.
     * @returns True if successful; false otherwise
     */
    clearContentFilter(): boolean;
  }
}
