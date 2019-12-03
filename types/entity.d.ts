
declare module 'rclnodejs' {

  /**
   *Class representing a common object in RCL.
   */
  class Entity {
    options: object;
    readonly qos: QoS;
    readonly typedArrayEnabled: boolean;
    readonly typeClass: TypeClass;
  }

}