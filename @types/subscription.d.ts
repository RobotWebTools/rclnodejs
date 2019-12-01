
declare module "rclnodejs" {

  /**
   * @class - Class representing a Subscription in ROS
   * @hideconstructor
   */
  class Subscription extends Entity {

    /**
     * @type {string}
     */
    readonly topic: string;
  }

}
