declare module 'rclnodejs' {
  namespace lifecycle {
    /**
     * A publisher that sends messages only when activated.
     * This implementation is based on the
     * {@link https://github.com/ros2/rclcpp/blob/master/rclcpp_lifecycle/include/rclcpp_lifecycle/lifecycle_publisher.hpp | rclcpp LifecyclePublisher class}
     */
    interface LifecyclePublisher<T extends TypeClass<MessageTypeClassName>>
      extends Publisher<T> {
      /**
       * Enables communications; publish() will now send messages.
       */
      activate(): void;

      /**
       * Disable communications; publish() will not send messages.
       */
      deactivate(): void;

      /**
       * Determine if communications are enabled, i.e., activated, or
       * disabled, i.e., deactivated.
       * @returns True if activated; otherwise false.
       */
      isActivated(): boolean;

      /**
       * Enables communications; publish() will now send messages.
       */
      onActivate(): void;

      /**
       * Disable communications; publish() will not send messages.
       */
      onDeactivate(): void;
    }
  }
}
