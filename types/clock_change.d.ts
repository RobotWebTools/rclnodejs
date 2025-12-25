declare module 'rclnodejs' {
  /**
   * Clock change type identifiers
   * Represents the type of clock change that occurred during a time jump.
   */
  enum ClockChange {
    /**
     * The source before and after the jump is ROS_TIME.
     */
    ROS_TIME_NO_CHANGE = 1,

    /**
     * The source switched to ROS_TIME from SYSTEM_TIME.
     */
    ROS_TIME_ACTIVATED = 2,

    /**
     * The source switched to SYSTEM_TIME from ROS_TIME.
     */
    ROS_TIME_DEACTIVATED = 3,

    /**
     * The source before and after the jump is SYSTEM_TIME.
     */
    SYSTEM_TIME_NO_CHANGE = 4,
  }
}
