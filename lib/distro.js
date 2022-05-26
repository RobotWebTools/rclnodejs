// Copyright (c) 2022 Wayne Parrott. All rights reserved.
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
 * enum style distribution identifiers
 */
const DistroId = {
  UNKNOWN: 0,
  ELOQUENT: 1911,
  FOXY: 2006,
  GALACTIC: 2105,
  HUMBLE: 2205,
  ROLLING: 5000,
};

const DistroNameIdMap = new Map();
DistroNameIdMap.set('eloquent', DistroId.ELOQUENT);
DistroNameIdMap.set('foxy', DistroId.FOXY);
DistroNameIdMap.set('galactic', DistroId.GALACTIC);
DistroNameIdMap.set('humble', DistroId.HUMBLE);
DistroNameIdMap.set('rolling', DistroId.ROLLING);

const DistroUtils = {
  DistroId: DistroId,

  /**
   * Get the rclnodejs distro ID for a ROS 2 distro name.
   * @param {string|undefined} [distroName] - The ROS 2 short distro name, e.g., foxy, Defaults to the value of the ROS_DISTRO envar.
   * @return {number} Return the rclnodejs distro identifier
   */
  getDistroId: function (distroName) {
    const dname = distroName ? distroName : this.getDistroName();

    return DistroNameIdMap.has(dname)
      ? DistroNameIdMap.get(dname)
      : DistroId.UNKNOWN;
  },

  /**
   * Get the short ROS 2 distro name associated with a rclnodejs distro ID.
   * @param {number|undefined} [distroId] - The rclnodejs distro identifier. Defaults to the value of the ROS_DISTRO envar.
   * @return {string|undefined} Return the name of the ROS distribution or undefined if unable to identify the distro.
   */
  getDistroName: function (distroId) {
    if (!distroId) {
      return process.env.ROS_DISTRO;
    }

    return [...DistroNameIdMap].find(([key, val]) => val == distroId)[0];
  },

  getKnownDistroNames: function () {
    return [...DistroNameIdMap.keys()];
  },
};

module.exports = DistroUtils;
