
declare module 'rclnodejs' {

  /**
   * Represents a ROS Time
   */
  class Time {

    /**
     * Create a Time.
     * @param {number|string} [seconds=0] - The second part of the time.
     * @param {number|string} [nanoseconds=0] - The nanosecond part of the time.
     * @param {ClockType} [clockType=Clock.ClockType.SYSTEM_TIME] - The clock type.
     */
    constructor(seconds?: number | string, nanoseconds?: number | string, clockType?: ClockType);

    /**
     * Get the the clock type of the Time object.
     * @name Time#get:clockType
     * @function
     * @return {ClockType} - The clock type.
     */

    readonly clockType: ClockType;

    /**
     * Get the nanosecond part of the time.
     * @name Time#get:nanoseconds
     * @function
     * @return {number|string} - value in nanosecond, if the value is greater than Number.MAX_SAFE_INTEGER (2^53-1), will be presented in string of decimal format.
     */
    readonly nanoseconds: number | string;

    /**
     * Get the time as separate seconds and nanoseconds component.
     * @name Time#get:secondsAndNanoseconds
     * @function
     * @return {object} - object with properties seconds and nanoseconds.
     */
    readonly secondsAndNanoseconds: { seconds: number; nanoseconds: number };

    /**
     * Add a duration to this time object.
     * @param {Duration} other - The Duration object to be added.
     * @return {Time} Return the result of a new Time object.
     */
    add(other: Duration): Time;

    /**
     * Subtract a duration/time to this time object.
     * @param {Duration|Time} other - The time to be subtracted.
     * @return {Duration|Time} Return the result.
     */
    sub(other: Duration | Time): Duration | Time;

    /**
     * Determine whether two Time objects are equal.
     * @param {Time} other - The time object to be compared.
     * @return {boolean} Return true if they are equal.
     */
    eq(other: Time): boolean;

    /**
     * Determine whether two Time objects are not equal.
     * @param {Time} other - The time object to be compared.
     * @return {boolean} Return true if they are not equal.
     */
    ne(other: Time): boolean;

    /**
     * Determine whether the time is less than another one.
     * @param {Time} other - The time object to be compared.
     * @return {boolean} Return true if it's less than other.
     */
    lt(other: Time): boolean;

    /**
     * Determine whether the time is less than or equal with another one.
     * @param {Time} other - The time object to be compared.
     * @return {boolean} Return true if it's less than or equal with other.
     */
    lte(other: Time): boolean;

    /**
     * Determine whether the time is greater than another one.
     * @param {Time} other - The time object to be compared.
     * @return {boolean} Return true if it's greater than other.
     */
    gt(other: Time): boolean;

    /**
     * Determine whether the time is greater than or equal with another one.
     * @param {Time} other - The time object to be compared.
     * @return {boolean} Return true if it's greater than or equal with other.
     */
    gte(other: Time): boolean;

    /**
     * Create a Time object from a message of builtin_interfaces/msg/Time
     * @param {object} msg - The message to be created from.
     * @param {ClockType} [clockType=Clock.ClockType.SYSTEM_TIME] - The type of the time object.
     * @return {Time} Return the created Time object.
     */
    static fromMsg(msg: object, clockType: ClockType): Time;
  }

}

