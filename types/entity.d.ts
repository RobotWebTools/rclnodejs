declare module 'rclnodejs' {
  /**
   * A common object in RCL.
   */
  interface Entity {
    options: object;
    readonly qos: QoS;
    readonly typedArrayEnabled: boolean;
    readonly typeClass: TypeClass;
  }
}
