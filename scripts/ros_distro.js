'use strict';

const DistroUtils = require('../lib/distro');

const distroName = DistroUtils.getDistroName();
const distroId = DistroUtils.getDistroId(distroName);

if (!distroName) {
  console.error(
    `Unable to detect ROS, please make sure a supported version of ROS is sourced.`
  );
  process.exit(1);
}

if (distroId === DistroUtils.UNKNOWN_ID) {
  console.error(`Unknown or unsupported ROS version "${distroName}"`);
  process.exit(1);
}

// otherwise
console.log(distroId);
process.exit(0);
