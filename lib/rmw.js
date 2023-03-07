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
