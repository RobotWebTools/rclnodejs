// Copyright (c) 2021 Wayne Parrott. All rights reserved.
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

const assert = require('assert');
const childProcess = require('child_process');
const fs = require('fs');
const os = require('os');
const path = require('path');
const rimraf = require('rimraf');
const rclnodejs = require('../index.js');

const GEN_FOLDER = 'generated';
const SCRIPT_NAME = 'generate-ros-messages';

// returns array of version fields [major, minor, maint]
function getNodeVersionInfo() {
  return process.version
    .substring(1)
    .split('.')
    .map((x) => parseInt(x));
}

describe('rclnodejs generate-messages binary-script tests', function () {
  let cwd;
  let tmpPkg;

  this.timeout(90 * 1000); // 90 seconds to run this test suite

  // Create a test pkg wtih rclnodejs dependency
  //   create test package folder
  //   run 'npm init' to create package.xml with rclnodejs as dependency
  //   run 'npm pack' on rclnodejs saving into test package
  //   in test package install from rclnodejs.x.y.z.tgz
  //   delete generated/ folder as this will be generated as part of a test
  before(function () {
    this.cwd = process.cwd(); // back up rclnodejs path
    let tmpDir = os.tmpdir();
    this.tmpPkg = path.join(tmpDir, 'rclnodejs_test');
    while (fs.existsSync(this.tmpPkg)) {
      this.tmpPkg += Math.trunc(Math.random() * 100);
    }

    fs.mkdirSync(this.tmpPkg);

    childProcess.spawnSync('npm', ['init', '-y'], {
      // stdio: 'inherit',
      shell: true,
      cwd: this.tmpPkg,
    });
    childProcess.spawnSync('npm', ['pack', this.cwd], {
      // stdio: 'inherit',
      shell: true,
      cwd: this.tmpPkg,
    });

    let tgz;
    const regex = /^rclnodejs-\d+.\d+.\d+.tgz/;
    for (let file of fs.readdirSync(this.tmpPkg)) {
      if (file.match(regex)) {
        tgz = file;
        break;
      }
    }
    if (!tgz) {
      console.error('ERROR: unable to successfully run npm pack');
      return;
    }
    let tgzPath = path.join(this.tmpPkg, tgz);
    childProcess.spawnSync('npm', ['install', tgzPath], {
      // stdio: 'inherit',
      shell: true,
      cwd: this.tmpPkg,
    });

    let generatedFolderPath = createGeneratedFolderPath(this.tmpPkg);
    if (fs.existsSync(generatedFolderPath)) {
      if (getNodeVersionInfo()[0] === 10) {
        rimraf.sync(generatedFolderPath);
      } else {
        fs.rmSync(generatedFolderPath, { recursive: true });
      }
    }
  });

  after(function () {
    // recursively remove test package folder
    if (getNodeVersionInfo()[0] === 10) {
      rimraf.sync(this.tmpPkg);
    } else {
      fs.rmSync(this.tmpPkg, { recursive: true });
    }
  });

  it('test generate-ros-messages script installation', function (done) {
    // confirm script is installed at <pgk>/node_modules/.bin/<script>
    let script = createScriptFolderPath(this.tmpPkg);
    assert.ok(fs.existsSync(script));
    done();
  });

  it('test generate-ros-messages script operation', function (done) {
    let script = createScriptFolderPath(this.tmpPkg);
    childProcess.spawnSync(script, [], {
      // stdio: 'inherit',
      shell: true,
    });

    let generatedFolderPath = createGeneratedFolderPath(this.tmpPkg);
    assert.ok(
      fs.existsSync(generatedFolderPath),
      'No generated message folder found'
    );
    assert.ok(
      fs.existsSync(path.join(generatedFolderPath, 'std_msgs')),
      'std_msgs folder found'
    );
    done();
  });

  it('test npx generate-ros-messages script operation', function (done) {
    childProcess.spawnSync('npx', [SCRIPT_NAME], {
      // stdio: 'inherit',
      shell: true,
      cwd: this.tmpPkg,
    });

    let generatedFolderPath = createGeneratedFolderPath(this.tmpPkg);
    assert.ok(
      fs.existsSync(generatedFolderPath),
      'No generated message folder found'
    );
    assert.ok(
      fs.existsSync(path.join(generatedFolderPath, 'std_msgs')),
      'std_msgs folder found'
    );
    done();
  });
});

function createGeneratedFolderPath(pkgFolder) {
  return path.join(pkgFolder, 'node_modules', 'rclnodejs', GEN_FOLDER);
}

function createScriptFolderPath(pkgFolder) {
  return path.join(pkgFolder, 'node_modules', '.bin', SCRIPT_NAME);
}
