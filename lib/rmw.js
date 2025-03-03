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

const DistroUtils = require('./distro');

const RMWNames = {
  FASTRTPS: 'rmw_fastrtps_cpp',
  CONNEXT: 'rmw_connext_cpp',
  CYCLONEDDS: 'rmw_cyclonedds_cpp',
  GURUMDDS: 'rmw_gurumdds_cpp',
};

const DefaultRosRMWNameMap = new Map();
DefaultRosRMWNameMap.set('eloquent', RMWNames.FASTRTPS);
DefaultRosRMWNameMap.set('foxy', RMWNames.FASTRTPS);
DefaultRosRMWNameMap.set('galactic', RMWNames.CYCLONEDDS);
DefaultRosRMWNameMap.set('humble', RMWNames.FASTRTPS);
DefaultRosRMWNameMap.set('rolling', RMWNames.FASTRTPS);

const RMWUtils = {
  RMWNames: RMWNames,

  getRMWName: function () {
    return process.env.RMW_IMPLEMENTATION
      ? process.env.RMW_IMPLEMENTATION
      : DefaultRosRMWNameMap.get(DistroUtils.getDistroName());
  },
};

module.exports = RMWUtils;
