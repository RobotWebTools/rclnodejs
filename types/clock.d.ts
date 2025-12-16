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
     * Add a clock callback.
     * @param callbackObject - The object containing _pre_callback and _post_callback methods.
     * @param onClockChange - Whether to call the callback on clock change.
     * @param minForward - Minimum forward jump to trigger the callback.
     * @param minBackward - Minimum backward jump to trigger the callback.
     */
    addClockCallback(
      callbackObject: object,
      onClockChange: boolean,
      minForward: bigint,
      minBackward: bigint
    ): void;

    /**
     * Remove a clock callback.
     * @param callbackObject - The object containing _pre_callback and _post_callback methods.
     */
    removeClockCallback(callbackObject: object): void;

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
