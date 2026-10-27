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

import assert from 'assert';
import childProcess from 'child_process';
import fs from 'fs';
import os from 'os';
import path from 'path';
import rclnodejs from '../index.js';

const GEN_FOLDER = 'generated';
const SCRIPT_NAME = 'generate-ros-messages';

// returns array of version fields [major, minor, maint]
function getNodeVersionInfo() {
  return process.version
    .substring(1)
    .split('.')
    .map((x) => parseInt(x));
}

function runCommand(command, args, options) {
  const result = childProcess.spawnSync(command, args, options);
  if (
    result.error ||
    result.signal ||
    (typeof result.status === 'number' && result.status !== 0)
  ) {
    const parts = [
      `Failed to run: ${command} ${Array.isArray(args) ? args.join(' ') : ''}`,
    ];
    if (
      result.error &&
      result.error.code === 'ETIMEDOUT' &&
      options &&
      options.timeout
    ) {
      parts.push(`Command timed out after ${options.timeout} ms`);
    } else if (result.error) {
      parts.push(`Error: ${result.error.message}`);
    }
    if (typeof result.status === 'number' && result.status !== 0) {
      parts.push(`Exit status: ${result.status}`);
    }
    if (result.signal) {
      parts.push(`Signal: ${result.signal}`);
    }
    if (result.stdout) {
      parts.push(`stdout:\n${result.stdout.toString()}`);
    }
    if (result.stderr) {
      parts.push(`stderr:\n${result.stderr.toString()}`);
    }
    throw new Error(parts.join('\n'));
  }
  return result;
}

// npm pack invokes tsdown, whose build-time Node range is narrower than ours.
const [nodeMajor, nodeMinor] = getNodeVersionInfo();
const supportsTsdown =
  (nodeMajor === 22 && nodeMinor >= 18) ||
  (nodeMajor === 24 && nodeMinor >= 11) ||
  nodeMajor >= 26;
const suite = supportsTsdown ? describe : describe.skip;

suite('rclnodejs generate-messages binary-script tests', function () {
  let cwd;
  let tmpPkg;
  const args = process.argv.find((arg) => arg === '--idl') ? ['--idl'] : [];

  this.timeout(15 * 60 * 1000);

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
      timeout: 60 * 1000,
    });
    runCommand('npm', ['pack', this.cwd], {
      // stdio: 'inherit',
      shell: true,
      cwd: this.tmpPkg,
      timeout: 120 * 1000,
    });

    let tgz;
    const regex = /^rclnodejs-\d+\.\d+\.\d+.*\.tgz/;
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
    runCommand('npm', ['install', tgzPath], {
      // stdio: 'inherit',
      shell: true,
      cwd: this.tmpPkg,
      timeout: 10 * 60 * 1000,
    });

    let generatedFolderPath = createGeneratedFolderPath(this.tmpPkg);
    if (fs.existsSync(generatedFolderPath)) {
      fs.rmSync(generatedFolderPath, { recursive: true });
    }
  });

  after(function () {
    // recursively remove test package folder
    fs.rmSync(this.tmpPkg, { recursive: true });
  });

  it('test generate-ros-messages script installation', function (done) {
    // confirm script is installed at <pgk>/node_modules/.bin/<script>
    let script = createScriptFolderPath(this.tmpPkg);
    assert.ok(fs.existsSync(script));
    done();
  });

  it('test generate-ros-messages script operation', function (done) {
    let script = createScriptFolderPath(this.tmpPkg);
    runCommand(script, args, {
      // stdio: 'inherit',
      shell: true,
      timeout: 120 * 1000,
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
    runCommand('npx', [SCRIPT_NAME, ...args], {
      // stdio: 'inherit',
      shell: true,
      cwd: this.tmpPkg,
      timeout: 120 * 1000,
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

  it('generates messages and declarations when the native addon must be built', function () {
    const packageRoot = path.join(this.tmpPkg, 'node_modules', 'rclnodejs');
    for (const folder of ['build', 'prebuilds', GEN_FOLDER]) {
      fs.rmSync(path.join(packageRoot, folder), {
        recursive: true,
        force: true,
      });
    }
    const declarations = path.join(packageRoot, 'types', 'interfaces.d.ts');
    fs.rmSync(declarations, { force: true });

    runCommand(createScriptFolderPath(this.tmpPkg), args, {
      shell: true,
      timeout: 5 * 60 * 1000,
    });

    assert.ok(
      fs.existsSync(
        path.join(packageRoot, 'build', 'Release', 'rclnodejs.node')
      ),
      'The CLI must build the missing native addon'
    );
    assert.ok(
      fs.existsSync(
        path.join(createGeneratedFolderPath(this.tmpPkg), 'std_msgs')
      ),
      'Generated messages must survive the native build'
    );
    assert.match(fs.readFileSync(declarations, 'utf8'), /namespace std_msgs/);
  });
});

function createGeneratedFolderPath(pkgFolder) {
  return path.join(pkgFolder, 'node_modules', 'rclnodejs', GEN_FOLDER);
}

function createScriptFolderPath(pkgFolder) {
  return path.join(pkgFolder, 'node_modules', '.bin', SCRIPT_NAME);
}
