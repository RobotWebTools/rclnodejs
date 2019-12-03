
declare module 'rclnodejs' {
  
  /**
    * A ROS Timer
    */
  class Timer {

    /**
     * @type {number}
     */
    readonly period: number;

    /**
     * Check if the timer is ready.
     * @return {boolean} Return true if timer is ready, otherwise return false.
     */
    isReady(): boolean;

    /**
     * Check if the timer is canceled.
     * @return {boolean} Return true if timer is canceled, otherwise return false.
     */
    isCanceled(): boolean;

    /**
     * Cancel the timer.
     * @return {undefined}
     */
    cancel(): void;

    /**
     * Reset the timer.
     * @return {undefined}
     */
    reset(): void;

    /**
     * Get the interval since the last call of this timer.
     * @return {number} - the interval value - ms.
     */
    timeSinceLastCall(): number;

    /**
     * Get the interval until the next call will happen.
     * @return {number} - the interval value - ms.
     */
    timeUntilNextCall(): number;
  }

}

