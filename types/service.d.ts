declare module 'rclnodejs' {
  /**
   * A ROS Service server that implements an interface.
   */
  class Service extends Entity {
    /**
     * Name of the service.
     */
    readonly serviceName: string;
  }
}
