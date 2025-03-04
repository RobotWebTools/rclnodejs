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

const fs = require('fs');
const path = require('path');
const os = require('os');

// blocklist.json format
//  [
//    {
//      pkgName: RegExString,
//      interfaceName: RegExString,
//      os: RegExString
//    },
//    ...
//  ]
//
// examples
//  [
//    {
//      "pkgName": "action*"
//    },
//    {
//      "pkgName": "std_msgs",
//    },
//    {
//      "pkgName": "std_msgs",
//      "interfaceName": "String"
//    },
//    {
//      "os": "Linux"
//    },
//  ]

const RosPackageFilters = {
  filters: [],
  _loaded: false,

  addFilter: function (pkgName, interfaceName, os) {
    this.filters.push({
      pkgName: pkgName,
      interfaceName: interfaceName,
      os: os,
    });
  },

  _matches: function (filter, pkgInfo) {
    if (filter.os && filter.os.test(os.type())) {
      return true;
    }

    if (filter.pkgName) {
      if (filter.pkgName.test(pkgInfo.pkgName)) {
        if (!filter.interfaceName) {
          return true;
        }
      } else {
        return false;
      }
    }

    if (
      filter.interfaceName &&
      filter.interfaceName.test(pkgInfo.interfaceName)
    ) {
      return true;
    }

    return false;
  },

  load: function (
    blocklistPath = path.join(__dirname, '../rosidl_gen/blocklist.json')
  ) {
    this._loaded = true;

    if (!fs.existsSync(blocklistPath)) return;

    let blocklistData = JSON.parse(fs.readFileSync(blocklistPath, 'utf8'));

    let filters = blocklistData.map((pkgFilterData) => {
      let filter = {};
      if (pkgFilterData['pkgName']) {
        filter.pkgName = new RegExp(pkgFilterData.pkgName);
      }
      if (pkgFilterData['interfaceName']) {
        filter.interfaceName = new RegExp(pkgFilterData.interfaceName);
      }
      if (pkgFilterData['os']) {
        filter.os = new RegExp(pkgFilterData.os);
      }
      return filter;
    });

    this.filters = filters.filter(
      (filter) => !filter.os || filter.os.test(os.type())
    );
  },

  matchesAny: function (pkgInfo) {
    if (!this._loaded) this.load();
    return this.filters.some((filter) => this._matches(filter, pkgInfo));
  },
};

module.exports = RosPackageFilters;
