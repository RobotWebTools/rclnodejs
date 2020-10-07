declare module 'rclnodejs' {
  /**
   * A ROS Timer that periodically executes a callback.
   */
  interface Timer {
    /**
     * Time between callbacks in milliseconds.
     */
    readonly period: number;

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
     * @returns The interval value in milliseconds.
     */
    timeSinceLastCall(): number;

    /**
     * Get the interval until the next call will happen.
     *
     * @returns The interval value in milliseconds
     */
    timeUntilNextCall(): number;
  }
}
