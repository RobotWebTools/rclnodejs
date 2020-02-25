declare module 'rclnodejs' {
  /**
   * A common object in RCL.
   */
  class Entity {
    options: object;
    readonly qos: QoS;
    readonly typedArrayEnabled: boolean;
    readonly typeClass: TypeClass;
  }
}
