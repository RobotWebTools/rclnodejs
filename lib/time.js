// Copyright (c) 2017 Intel Corporation. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

'use strict';

const rclnodejs = require('bindings')('rclnodejs');

/* eslint-disable camelcase */

const RCL_ROS_TIME = 1;
const RCL_SYSTEM_TIME = 2;
const RCL_STEADY_TIME = 3;

function toNumberType(s) {
  if (s === 'ros-clock') {
    return RCL_ROS_TIME;
  } else if (s === 'system-clock') {
    return RCL_SYSTEM_TIME;
  } else if (s === 'steady-clock-clock') {
    return RCL_STEADY_TIME;
  }
  return 0;
}

function toStringType(n) {
  if (n == RCL_ROS_TIME) {
    return 'ros-clock';
  } else if (n == RCL_SYSTEM_TIME) {
    return 'system-clock';
  } else if (n == RCL_STEADY_TIME) {
    return 'steady-clock';
  }
  return 'invalid-clock';
}

class Time {
  constructor(type) {
    this._handle = rclnodejs.createClock(toNumberType(type));
    this._type = type;
  }

  get type() {
    return this._type;
  }

  get handle() {
    return this._handle;
  }

  now() {
    return rclnodejs.clockGetNow(this._handle);
  }

  static rosNow() {
    return rclnodejs.staticClockGetNow(RCL_ROS_TIME);
  }

  static systemNow() {
    return rclnodejs.staticClockGetNow(RCL_SYSTEM_TIME);
  }

  static steadyNow() {
    return rclnodejs.staticClockGetNow(RCL_STEADY_TIME);
  }

  static diff(start, finish) {
    return rclnodejs.timeDiff(start.sec, start.nanosec, toNumberType(start.type),
      finish.sec, finish.nanosec, toNumberType(finish.type));
  }
}

module.exports = {
  ros_clock: 'ros-clock',
  system_clock: 'system-clock',
  steady_clock: 'steady-clock',

  RCL_ROS_TIME: RCL_ROS_TIME,
  RCL_SYSTEM_TIME: RCL_SYSTEM_TIME,
  RCL_STEADY_TIME: RCL_STEADY_TIME,

  toNumberType: toNumberType,
  toStringType: toStringType,

  Time: Time,
};
