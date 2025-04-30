declare module 'rclnodejs' {
  /**
   * A ROS Timer that periodically executes a callback.
   */
  interface Timer {
    /**
     * Time between callbacks in nanoseconds.
     */
    readonly period: bigint;

    /**
     * Check if the timer is ready.
     *
     * @returns True if timer is ready, otherwise return false.
     */
    isReady(): boolean;

    /**
     * Check if the timer has been cancelled.
     *
     * @returns True if timer has been canceled, otherwise return false.
     */
    isCanceled(): boolean;

    /**
     * Cancel the timer.
     */
    cancel(): void;

    /**
     * Reset the timer.
     */
    reset(): void;

    /**
     * Get the interval since the last call of this timer.
     *
     * @returns The interval value in nanoseconds.
     */
    timeSinceLastCall(): bigint;

    /**
     * Get the interval until the next call will happen.
     *
     * @returns The interval value in nanoseconds
     */
    timeUntilNextCall(): bigint;

    /**
     * Change the timer period.
     * @param period - The new period in nanoseconds.
     */
    changeTimerPeriod(period: bigint): void;

    /**
     * Get the timer period.
     * @return The period in nanoseconds.
     */
    timerPeriod(): bigint;

    /**
     * Call a timer and starts counting again, retrieves actual and expected call time.
     * @return - The timer information.
     */
    callTimerWithInfo(): object;
  }
}
