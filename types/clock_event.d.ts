declare module 'rclnodejs' {
  /**
   * Class representing a ClockEvent in ROS
   */
  class ClockEvent {
    /**
     * Create a ClockEvent.
     */
    constructor();

    /**
     * Wait until a time specified by a steady clock.
     * @param clock - The clock to use for time synchronization.
     * @param until - The time to wait until.
     * @returns A promise that resolves when the time is reached.
     */
    waitUntilSteady(clock: Clock, until: bigint): Promise<void>;

    /**
     * Wait until a time specified by a system clock.
     * @param clock - The clock to use for time synchronization.
     * @param until - The time to wait until.
     * @returns A promise that resolves when the time is reached.
     */
    waitUntilSystem(clock: Clock, until: bigint): Promise<void>;

    /**
     * Wait until a time specified by a ROS clock.
     * @param clock - The clock to use for time synchronization.
     * @param until - The time to wait until.
     * @returns A promise that resolves when the time is reached.
     */
    waitUntilRos(clock: Clock, until: bigint): Promise<void>;

    /**
     * Indicate if the ClockEvent is set.
     * @returns True if the ClockEvent is set.
     */
    isSet(): boolean;

    /**
     * Set the event.
     */
    set(): void;

    /**
     * Clear the event.
     */
    clear(): void;
  }
}
