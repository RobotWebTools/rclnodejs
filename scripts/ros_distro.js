'use strict';

switch (process.env.ROS_DISTRO) {
  case 'eloquent':
    console.log('1911');
    process.exit(0);
  case 'foxy':
  case 'rolling':
    console.log('2006');
    process.exit(0);
  case undefined:
    console.error(
      'Unable to detect ROS, please make sure a supported version of ROS is sourced'
    );
    process.exit(1);
  default:
    console.error(
      `Unknown or unsupported ROS version "${process.env.ROS_DISTRO}"`
    );
    process.exit(1);
}
