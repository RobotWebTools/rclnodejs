
declare module 'rclnodejs' {

  /**
   * ROS Clock.
   */
  class Clock {
    /**
     * Create a Clock.
     * @param clockType - Type of the clock to create; default = ClockType.SYSTEM_TIME.
     */
    constructor(clockType?: ClockType);

    /**
     * Get ClockType of this Clock object.
     * 
     * @return  Type of this clock.
     */
    readonly clockType: ClockType;

    /**
     * Return the current time.
     * 
     * @return Return the current time.
     */
    now(): Time;
  }

  /**
   * A ROSClock. 
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
