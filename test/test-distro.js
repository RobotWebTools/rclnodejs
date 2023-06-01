'use strict';

const assert = require('assert');
const childProcess = require('child_process');
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
      6,
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
