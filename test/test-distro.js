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

const assert = require('assert');
const rclnodejs = require('../index.js');

const DistroUtils = rclnodejs.DistroUtils;

describe('rclnodejs distro utils', function () {
  it('Valid distro names', function (done) {
    let backupEnvar = process.env.ROS_DISTRO;

    const distroNames = DistroUtils.getKnownDistroNames();
    assert.ok(
      distroNames,
      'DistroUtils.getKnownDistroNames() did not return any distro names'
    );
    assert.equal(
      distroNames.length,
      8,
      'Incorrect number of known distro names'
    );

    distroNames.forEach((distroName) => {
      let id = DistroUtils.getDistroId(distroName);
      assert.ok(id, `Unknown distro name: ${distroName}`);

      // test defaults from env
      process.env.ROS_DISTRO = distroName;
      assert.equal(
        distroName,
        DistroUtils.getDistroName(),
        'Invalid ROS_DISTRO envar test'
      );

      id = DistroUtils.getDistroId();
      assert.notEqual(id, DistroUtils.DistroId.UNKNOWN);
      assert.equal(id, DistroUtils.getDistroId());

      id = DistroUtils.getDistroId(distroName);
      assert.notEqual(id, DistroUtils.UNKNOWN_ID);
      assert.equal(distroName, DistroUtils.getDistroName(id));
    });

    process.env.ROS_DISTRO = backupEnvar;
    done();
  });

  it('unknown distro', function (done) {
    let backupEnvar = process.env.ROS_DISTRO;

    // test unknown distro
    process.env.ROS_DISTRO = 'xxx';
    assert.equal(
      'xxx',
      DistroUtils.getDistroName(),
      `Failed unknown distro name`
    );
    let id = DistroUtils.getDistroId();
    assert.equal(id, DistroUtils.DistroId.UNKNOWN);
    assert.equal(
      DistroUtils.DistroId.UNKNOWN,
      DistroUtils.getDistroId('xxx'),
      "getDistroId('xxx') failed"
    );

    process.env.ROS_DISTRO = backupEnvar;
    done();
  });
});
