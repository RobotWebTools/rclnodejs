// Copyright (c) 2025 The Robot Web Tools Contributors. All rights reserved.
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

/**
 * Enum for ClockChange
 * Represents the type of clock change that occurred during a time jump.
 * @readonly
 * @enum {number}
 */
const ClockChange = {
  /**
   * The source before and after the jump is ROS_TIME.
   * @member {number}
   */
  ROS_TIME_NO_CHANGE: 1,

  /**
   * The source switched to ROS_TIME from SYSTEM_TIME.
   * @member {number}
   */
  ROS_TIME_ACTIVATED: 2,

  /**
   * The source switched to SYSTEM_TIME from ROS_TIME.
   * @member {number}
   */
  ROS_TIME_DEACTIVATED: 3,

  /**
   * The source before and after the jump is SYSTEM_TIME.
   * @member {number}
   */
  SYSTEM_TIME_NO_CHANGE: 4,
};

module.exports = ClockChange;
