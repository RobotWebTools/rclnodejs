// Copyright (c) 2017 Intel Corporation. All rights reserved.
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
const fs = require('fs');
const os = require('os');
const path = require('path');
const childProcess = require('child_process');

function assertMember(name, obj, member, typeName) {
  assert.ok(name in obj);
  assert.deepStrictEqual(typeof member, typeName);
}

function assertThrowsError(operation, errors, errMsg, message) {
  assert.throws(
    operation,
    function(err) {
      var containedMsg = new RegExp(errMsg);

      if (errors instanceof Array) {
        var foundError = false;
        errors.forEach(e => {
          if (err instanceof e) foundError = true;
        });
        return foundError;
      } else if (err instanceof errors && containedMsg.test(err)) {
        return true;
      }

      return false;
    },
    message
  );
}

function launchPythonProcess(cmdline) {
  var pythonProcess = null;
  if (os.platform() === 'win32') {
    cmdline.unshift('-3.7');
    pythonProcess = childProcess.spawn('py', cmdline);
  } else {
    pythonProcess = childProcess.spawn('python3', cmdline);
  }
  return pythonProcess;
}

function getAvailablePath(amentPrefixPath, otherDirs) {
  var availablePath;
  var prefixPaths = amentPrefixPath.split(path.delimiter);

  prefixPaths.forEach(prefixPath => {
    var appendedPath = prefixPath;
    otherDirs.forEach(dir => {
      appendedPath = path.join(appendedPath, dir);
    });

    // eslint-disable-next-line
    if (fs.existsSync(appendedPath) || fs.existsSync(appendedPath + '.exe')) {
      availablePath = appendedPath;
    }
  });

  return availablePath;
}

// example call from async function/method:
//
//  await assertUtils.createDelay(500);
//
function createDelay(millis) {
  return new Promise(resolve => setTimeout(resolve, millis));
}

module.exports = {
  assertMember: assertMember,
  assertThrowsError: assertThrowsError,
  createDelay: createDelay,
  getAvailablePath: getAvailablePath,
  launchPythonProcess: launchPythonProcess,
};
