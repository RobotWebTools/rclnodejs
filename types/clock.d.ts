declare module 'rclnodejs' {
  /**
   * Jump information provided to clock jump callbacks.
   */
  interface ClockJumpInfo {
    /**
     * Type of clock change that occurred.
     */
    clock_change: ClockChange;

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

    /**
     * Sleep until a specific time is reached on this Clock.
     *
     * When using a ROSClock, this may sleep forever if the TimeSource is misconfigured
     * and the context is never shut down. ROS time being activated or deactivated causes
     * this function to cease sleeping and return false.
     *
     * @param until - Time at which this function should stop sleeping.
     * @param context - Context which when shut down will cause this sleep to wake early.
     *   If context is null or undefined, then the default context is used.
     * @returns Promise that resolves to true if 'until' was reached,
     *   or false if it woke for another reason (time jump, context shutdown).
     * @throws {TypeError} if until is specified for a different type of clock than this one.
     * @throws {Error} if context has not been initialized or is shutdown.
     */
    sleepUntil(until: Time, context?: Context | null): Promise<boolean>;

    /**
     * Sleep for a specified duration.
     *
     * Equivalent to: clock.sleepUntil(clock.now() + duration, context)
     *
     * When using a ROSClock, this may sleep forever if the TimeSource is misconfigured
     * and the context is never shut down. ROS time being activated or deactivated causes
     * this function to cease sleeping and return false.
     *
     * @param duration - Duration of time to sleep for.
     * @param context - Context which when shut down will cause this sleep to wake early.
     *   If context is null or undefined, then the default context is used.
     * @returns Promise that resolves to true if the full duration was slept,
     *   or false if it woke for another reason.
     * @throws {Error} if context has not been initialized or is shutdown.
     */
    sleepFor(duration: Duration, context?: Context | null): Promise<boolean>;
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
