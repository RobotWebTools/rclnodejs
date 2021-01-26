'use strict';

const useRosIdl =
  eval(process.env.RCLNODEJS_USE_ROSIDL) ||
  eval(process.env.npm_config_rclnodejs_use_rosidl);

module.exports = {
  useRosIdl,
};
