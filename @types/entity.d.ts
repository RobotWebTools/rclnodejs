
declare module "rclnodejs" {

  /**
   * @class - Class representing a common object in RCL.
   * @ignore
   */
  class Entity {
    options: object;
    readonly qos: QoS;
    readonly typedArrayEnabled: boolean;
    readonly typeClass: TypeClass;
  }

}