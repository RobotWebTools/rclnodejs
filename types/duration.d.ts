declare module 'rclnodejs' {
  /**
   * A ROS Duration is a measure of elapsed time.
   * A duration consists of 2 components: seconds and nanoseconds.
   */
  class Duration {
    /**
     * Create a Duration.
     *
     * @param seconds - The seconds component of the duration, default = 0.
     * @param nanoseconds - The nanoseconds component of the duration, default = 0.
     */
    constructor(seconds?: number | string, nanoseconds?: number | string);

    /**
     * Get the nanosecond component of the Duration.
     *
     * @returns The nanoseconds, if the value is greater than Number.MAX_SAFE_INTEGER (2^53-1),
     *         will be presented in a string of decimal format.
     */
    readonly nanoseconds: number | string;

    /**
     * Test if this Duration is equal to another Duration.
     *
     * @param other - The Duration this is compare to.
     * @returns True if this duration is equal to other duration.
     */
    eq(other: Duration): boolean;

    /**
     * Test if this Duration is not equal to another Duration.
     *
     * @param other - The Duration this is compare to.
     * @returns True if this duration is not equal to other duration.
     */
    ne(other: Duration): boolean;

    /**
     * Test if this Duration is less than another Duration.
     *
     * @param other - The Duration this is compare to.
     * @returns True if this duration is less than other duration.
     */
    lt(other: Duration): boolean;

    /**
     * Test if this Duration is less than or equal to another Duration.
     *
     * @param other - The Duration this is compare to.
     * @returns True if this duration is less than or equal to other duration.
     */
    lte(other: Duration): boolean;

    /**
     * Test if this Duration is greater than another Duration.
     *
     * @param other - The Duration this is compare to.
     * @returns True if this duration is greater than other duration.
     */
    gt(other: Duration): boolean;

    /**
     * Test if this Duration is greater than or equal another Duration.
     *
     * @param other - The Duration this is compare to.
     * @returns True if this duration is greater than or equal to other duration.
     */
    gte(other: Duration): boolean;
  }
}
