
declare module "rclnodejs" {

  /**
   * @class - Class representing a Clock in ROS
   */
  class Clock {
    /**
     * Create a Clock.
     * @param {ClockType} [clockType=ClockType.SYSTEM_TIME] - The type of the clock to be created.
     */
    constructor(clockType?: ClockType);

    /**
     * Get ClockType of this Clock object.
     * @return {ClockType} Return the type of the clock.
     */
    readonly clockType: ClockType;

    /**
     * Return the current time.
     * @return {Time} Return the current time.
     */
    now(): Time;
  }

  /**
   * @class - Class representing a ROSClock in ROS
   */

  class ROSClock extends Clock {
    /**
     * Create a ROSClock.
     */
    constructor();

    /**
     * Status that whether the ROS time is active.
     * @name ROSClock#get:isRosTimeActive
     */
    isRosTimeActive: boolean;


    /**
     * Status of ROS time.
     */

    rosTimeOverride: Time;
  }

}
