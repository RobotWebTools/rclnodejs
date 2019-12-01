
declare module "rclnodejs" {

  /**
   * @class - Class representing a Publisher in ROS
   * @hideconstructor
   */
  class Publisher extends Entity {

    /**
     * @type {string}
     */
    readonly topic: string;

    /**
     * Publish a message
     * @param {object} message - The message to be sent.
     * @return {undefined}
     */
    publish(message: string|object): void;
  }

}