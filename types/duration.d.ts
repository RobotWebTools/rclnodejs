
declare module 'rclnodejs' {

  /**
   * @class - Class representing a Duration in ROS
   */

  class Duration {
    /**
     * Create a Duration.
     * @param {number|string} [seconds=0] - The second part of the duration.
     * @param {number|string} [nanoseconds=0] - The nanosecond part of the duration.
     */
    constructor(seconds?: number | string, nanoseconds?: number | string);

    /**
     * Get the nanosecond part of the Duration.
     * @name Duration#get:nanoseconds
     * @function
     * @return {number|string} - value in nanosecond, if the value is greater than Number.MAX_SAFE_INTEGER (2^53-1), will be presented in string of decimal format.
     */

    readonly nanoseconds: number | string;

    /**
     * Determine whether two Duration objects are equal.
     * @param {Duration} other - The Duration object to be compared.
     * @return {boolean} Return true if they are equal.
     */
    eq(other: Duration): boolean;

    /**
     * Determine whether two Duration objects are not equal.
     * @param {Duration} other - The Duration object to be compared.
     * @return {boolean} Return true if they are not equal.
     */
    ne(other: Duration): boolean;

    /**
     * Determine whether the Duration object is less than another one.
     * @param {Duration} other - The Duration object to be compared.
     * @return {boolean} Return true if it's less than other.
     */
    lt(other: Duration): void;

    /**
     * Determine whether the Duration object is less than or equal with another one.
     * @param {Duration} other - The Duration object to be compared.
     * @return {boolean} Return true if it's less than or equal with other.
     */
    lte(other: Duration): void;

    /**
     * Determine whether the Duration object is greater than another one.
     * @param {Duration} other - The Duration object to be compared.
     * @return {boolean} Return true if it's greater than other.
     */
    gt(other: Duration): boolean;

    /**
     * Determine whether the Duration object is greater than or equal with another one.
     * @param {Duration} other - The Duration object to be compared.
     * @return {boolean} Return true if it's greater than or equal with other.
     */
    gte(other: Duration): boolean;
  }

}

