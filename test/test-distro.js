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

import assert from 'assert';
import { spawnSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import rclnodejs from '../index.js';

const DistroUtils = rclnodejs.DistroUtils;

describe('rclnodejs distro utils', function () {
  it('Valid distro names', function (done) {
    let backupEnvar = process.env.ROS_DISTRO;

    const distroNames = DistroUtils.getKnownDistroNames();
    assert.ok(
      distroNames && distroNames.length > 0,
      'DistroUtils.getKnownDistroNames() did not return any distro names'
    );

    const expectedDistros = [
      'eloquent',
      'foxy',
      'galactic',
      'humble',
      'iron',
      'jazzy',
      'kilted',
      'lyrical',
      'rolling',
    ];
    for (const name of expectedDistros) {
      assert.ok(
        distroNames.includes(name),
        `Expected distro '${name}' not found in known distro names`
      );
    }

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
      assert.notEqual(id, DistroUtils.DistroId.UNKNOWN);
      assert.equal(distroName, DistroUtils.getDistroName(id));
    });

    process.env.ROS_DISTRO = backupEnvar;
    done();
  });

  it('unknown distro', function (done) {
    let backupEnvar = process.env.ROS_DISTRO;

    // test unknown distro — when ROS_DISTRO is set but unrecognized,
    // assume it is a future distro (newer than Rolling)
    process.env.ROS_DISTRO = 'xxx';
    assert.equal(
      'xxx',
      DistroUtils.getDistroName(),
      `Failed unknown distro name`
    );
    let id = DistroUtils.getDistroId();
    assert.equal(id, DistroUtils.DistroId.FUTURE);
    assert.equal(
      DistroUtils.DistroId.FUTURE,
      DistroUtils.getDistroId('xxx'),
      "getDistroId('xxx') failed"
    );

    // test truly unknown — ROS_DISTRO unset
    delete process.env.ROS_DISTRO;
    assert.equal(DistroUtils.getDistroId(), DistroUtils.DistroId.UNKNOWN);

    if (backupEnvar !== undefined) {
      process.env.ROS_DISTRO = backupEnvar;
    }
    done();
  });

  it('getDistroName with unknown id returns undefined', function () {
    assert.strictEqual(
      DistroUtils.getDistroName(99999),
      undefined,
      'getDistroName(99999) should return undefined, not throw'
    );
  });
});

describe('ROS distro build script', function () {
  const script = fileURLToPath(
    new URL('../scripts/ros_distro.cjs', import.meta.url)
  );

  function runScript(distroName, args = [script]) {
    const env = { ...process.env };
    // Keep LSan's suppression statistics out of stderr without disabling checks.
    env.LSAN_OPTIONS = [env.LSAN_OPTIONS, 'print_suppressions=0']
      .filter(Boolean)
      .join(':');
    if (distroName === undefined) {
      delete env.ROS_DISTRO;
    } else {
      env.ROS_DISTRO = distroName;
    }
    const result = spawnSync(process.execPath, args, {
      env,
      encoding: 'utf8',
      timeout: 10000,
    });
    assert.ifError(result.error);
    return result;
  }

  for (const distroName of [undefined, '']) {
    it(`rejects ${distroName === undefined ? 'unset' : 'empty'} ROS_DISTRO`, function () {
      const result = runScript(distroName);
      assert.strictEqual(result.status, 1);
      assert.strictEqual(result.stdout, '');
      assert.match(result.stderr, /Unable to detect ROS/);
    });
  }

  for (const [distroName, id] of [
    ['humble', 2205],
    ['jazzy', 2405],
    ['kilted', 2505],
    ['lyrical', 2605],
    ['rolling', 5000],
    ['foxy', 2006],
    ['future_test_distro', 9999],
  ]) {
    it(`preserves the existing ID for ${distroName}`, function () {
      const result = runScript(distroName);
      assert.strictEqual(result.status, 0, result.stderr);
      assert.strictEqual(result.stdout.trim(), String(id));
      assert.strictEqual(result.stderr, '');
    });
  }

  it('rejects the UNKNOWN enum value when the distro name is present', function () {
    const distroModule = fileURLToPath(
      new URL('../lib/distro.js', import.meta.url)
    );
    const result = runScript('humble', [
      '-e',
      `
    const distro = require(process.argv[1]).default;
    distro.getDistroId = () => distro.DistroId.UNKNOWN;
    require(process.argv[2]);
    `,
      distroModule,
      script,
    ]);
    assert.strictEqual(result.status, 1);
    assert.strictEqual(result.stdout, '');
    assert.match(result.stderr, /Unknown or unsupported ROS version "humble"/);
  });
});
