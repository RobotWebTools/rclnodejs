declare module 'rclnodejs' {
  /**
   * A utility class for inspecting ROS 2 message structure without using loader.loadInterface directly.
   * Provides access to message schema, field names, and default values.
   */
  export class MessageIntrospector<T = any> {
    /**
     * Create a new MessageIntrospector for a ROS 2 message type.
     * @param typeName - The full message type name (e.g., 'geometry_msgs/msg/Twist')
     * @throws {TypeValidationError} If typeName is not a non-empty string
     * @throws {Error} If the message type cannot be loaded
     */
    constructor(typeName: string);

    /**
     * Get the full message type name.
     */
    readonly typeName: string;

    /**
     * Get the underlying ROS message class.
     */
    readonly typeClass: new () => T;

    /**
     * Get the field names of the message.
     */
    readonly fields: string[];

    /**
     * Get the ROSMessageDef schema for the message type.
     */
    readonly schema: ROSMessageDef;

    /**
     * Get the default values for all fields.
     * Creates a new instance of the message and converts it to a plain object.
     * Result is cached for performance.
     */
    readonly defaults: T;
  }

  /**
   * ROSMessageDef schema structure.
   */
  interface ROSMessageDef {
    constants: Array<{
      name: string;
      type: string;
      value: any;
    }>;
    fields: Array<{
      name: string;
      type: {
        isPrimitiveType: boolean;
        type: string;
        pkgName: string | null;
        isArray: boolean;
        isDynamicArray: boolean;
        isFixedSizeArray: boolean | null;
        arraySize: number | null;
        stringUpperBound: number | null;
        isUpperBound: boolean;
      };
      default_value: any;
    }>;
    msgName: string;
    baseType: {
      pkgName: string;
      type: string;
      stringUpperBound: number | null;
      isPrimitiveType: boolean;
    };
  }
}
