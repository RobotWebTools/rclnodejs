import { MessageTypeClassName } from 'rclnodejs';

declare module 'rclnodejs' {
  /**
   * A ROS Publisher that publishes messages on a topic.
   */
  class Publisher<T extends TypeClass<MessageTypeClassName>> extends Entity {
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
  }
}
