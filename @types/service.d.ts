
declare module "rclnodejs" {

  /**
   * @class - Class representing a Service in ROS
   * @hideconstructor
   */
  class Service extends Entity {

    /**
     * @type {string}
     */
    readonly serviceName: string;
  }

}