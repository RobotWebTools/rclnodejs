declare module 'rclnodejs' {
  /**
   * @class - Represents a unique identifier used by actions.
   * @ignore
   */

  export class ActionUuid {
    /**
     * Creates a new instance of ActionUuid.
     * @param bytes - The bytes to create the UUID from.
     *                A new random UUID will be created, if not provided.
     */
    constructor(bytes?: Uint8Array);

    /**
     * Creates a new {@link ActionUuid} from the given bytes.
     * @param bytes - The bytes to create the UUID from.
     * @returns The new UUID.
     */
    static fromBytes(byte: Uint8Array): ActionUuid;

    /**
     * Creates a new random {@link ActionUuid}.
     * @returns The new UUID.
     */
    static random(): ActionUuid;

    /**
     * Gets the bytes from the UUID.
     */
    get bytes(): Uint8Array;

    /**
     * Returns the UUID as a string.
     * @returns String representation of the UUID.
     */
    toString(): string;

    /**
     * Create an instance from a ROS2 UUID message
     * @param msg - The ROS2 UUID message
     * @returns The new instance.
     */
    static fromMessage(msg: unique_identifier_msgs.msg.UUID): ActionUuid;

    /**
     * Create a ROS2 UUID message from this instance.
     * @returns The new ROS2 UUID message
     */
    toMessage(): unique_identifier_msgs.msg.UUID;

    /**
     * Create a ROS2 UUID message with random uuid data
     * @returns The new instance.
     */
    static randomMessage(): unique_identifier_msgs.msg.UUID;
  }
}
