declare module 'rclnodejs' {
  /**
   * Jump information provided to clock jump callbacks.
   */
  interface ClockJumpInfo {
    /**
     * Type of clock change that occurred.
     */
    clock_change: number;

    /**
     * Time delta in nanoseconds.
     */
    delta: bigint;
  }

  /**
   * Callback object for clock jump events.
   */
  interface ClockCallbackObject {
    /**
     * Optional callback invoked before a time jump.
     */
    _pre_callback?: () => void;

    /**
     * Optional callback invoked after a time jump.
     * @param jumpInfo - Information about the time jump.
     */
    _post_callback?: (jumpInfo: ClockJumpInfo) => void;
  }

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
     * @param callbackObject - The object containing callback methods.
     * @param onClockChange - Whether to call the callback on clock change.
     * @param minForward - Minimum forward jump in nanoseconds to trigger the callback.
     * @param minBackward - Minimum backward jump in nanoseconds to trigger the callback.
     */
    addClockCallback(
      callbackObject: ClockCallbackObject,
      onClockChange: boolean,
      minForward: bigint,
      minBackward: bigint
    ): void;

    /**
     * Remove a clock callback.
     * @param callbackObject - The callback object that was previously registered with addClockCallback().
     */
    removeClockCallback(callbackObject: ClockCallbackObject): void;

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
