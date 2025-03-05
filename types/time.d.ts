import { builtin_interfaces } from 'rclnodejs';

declare module 'rclnodejs' {
  /**
   * Represents a ROS Time
   */
  class Time {
    /**
     * Create a Time.
     *
     * @param seconds - The seconds component of the time, default = 0.
     * @param nanoseconds - The nanoseconds component of the time, default = 0.
     * @param clockType - The clock type, default = Clock.ClockType.SYSTEM_TIME
     */
    constructor(seconds?: bigint, nanoseconds?: bigint, clockType?: ClockType);

    /**
     * Get the the clock type of the Time object.
     */
    readonly clockType: ClockType;

    /**
     * Get the nanosecond part of the time.
     */
    readonly nanoseconds: bigint;

    /**
     * Get the time as a plain JavaScript object.
     */
    readonly secondsAndNanoseconds: { seconds: bigint; nanoseconds: bigint };

    /**
     * Add a duration to this time object.
     *
     * @param other - The Duration object to be added.
     * @returns The sum of this and other duration.
     */
    add(other: Duration): Time;

    /**
     * Subtract a duration or time from this time object.
     *
     * @param other - The time to be subtracted.
     * @returns The difference between this and other time.
     */
    sub(other: Duration | Time): Duration | Time;

    /**
     * Test if this Time is equal to another Time.
     *
     * @param other - The Time this is compare to.
     * @returns True if this time is equal to other time.
     */
    eq(other: Time): boolean;

    /**
     * Test if this Time is not equal to another Time.
     *
     * @param other - The Time this is compare to.
     * @returns True if this time is not equal to other time.
     */
    ne(other: Time): boolean;

    /**
     * Test if this Time is less than another Time.
     *
     * @param other - The Time this is compare to.
     * @returns True if this time is less than other time.
     */
    lt(other: Time): boolean;

    /**
     * Test if this Time is less than or equal to another Time.
     *
     * @param other - The Time this is compare to.
     * @returns True if this time is less than or equal to other time.
     */
    lte(other: Time): boolean;

    /**
     * Test if this Time is greater than another Time.
     *
     * @param other - The Time this is compare to.
     * @returns True if this time is greater than other time.
     */
    gt(other: Time): boolean;

    /**
     * Test if this Time is greater than or equal to another Time.
     *
     * @param other - The Time this is compare to.
     * @returns True if this time is greater than or equal to other time.
     */
    gte(other: Time): boolean;

    /**
     * Create a builtin_interfaces/msg/Time message
     *
     * @returns The new Time message.
     */
    toMsg(): builtin_interfaces.msg.Time;

    /**
     * Create a Time object from a message of builtin_interfaces/msg/Time
     *
     * @param msg - The Time message to be created from.
     * @param clockType - The type of the time object. Default is ClockType.ROS_TIME
     * @returns The new Time.
     */
    static fromMsg(
      msg: builtin_interfaces.msg.Time,
      clockType?: ClockType
    ): Time;
  }
}
