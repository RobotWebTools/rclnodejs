declare module 'rclnodejs' {
  /**
   * A ROS Clock.
   */
  class Clock {
    /**
     * Create a Clock.
     *
     * @param clockType - Type of the clock to create; default = {@link ClockType.SYSTEM_TIME}.
     */
    constructor(clockType?: ClockType);

    /**
     * Get ClockType of this Clock.
     *
     * @returns Type of this clock.
     */
    readonly clockType: ClockType;

    /**
     * Return the current time.
     *
     * @returns The current time.
     */
    now(): Time;
  }

  /**
   * A ROS ROSClock.
   */
  class ROSClock extends Clock {
    /**
     * Create a ROSClock.
     */
    constructor();

    /**
     * Determine if the clock is active.
     */
    isRosTimeActive: boolean;

    /**
     * Status of ROS time.
     */
    rosTimeOverride: Time;
  }
}
