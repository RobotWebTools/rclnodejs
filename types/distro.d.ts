declare module 'rclnodejs' {
  namespace DistroUtils {
    /**
     * Valid ROS 2 distro short names
     */
    type DistroName = 'eloquent' | 'foxy' | 'galactic' | 'humble' | 'rolling';

    /**
     * rclnodejs distro ID numbers
     */
    enum DistroId {
      UNKNOWN = 0,
      ELOQUENT = 1911,
      FOXY = 2006,
      GALACTIC = 2105,
      HUMBLE = 2205,
      ROLLING = 5000,
    }

    /**
     * Get the rclnodejs distro ID for a ROS 2 distro name.
     * @param distroName - The ROS 2 short distro name, e.g., foxy, Defaults to the value of the ROS_DISTRO envar.
     * @return The rclnodejs distro identifier
     */
    function getDistroId(distroName?: DistroName): DistroId;

    /**
     * Get the short ROS 2 distro name associated with a rclnodejs distro ID.
     * @param distroId - The rclnodejs distro identifier. Defaults to the value of the ROS_DISTRO envar.
     * @returns The name of the ROS distribution or undefined if unable to identify the distro.
     */
    function getDistroName(distroId?: DistroId): string | undefined;
  }
}
