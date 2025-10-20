// Copyright (c) 2025, The Robot Web Tools Contributors
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

const fs = require('fs');
const path = require('path');
const fse = require('../lib/utils.js');
const execFile = require('child_process').execFile;
const pythonExecutable =
  require('../rosidl_parser/py_utils').getPythonExecutable('python3');

async function convertIDLToROS2IDL(pkgName, idlFilePath, outputDir) {
  const packagePath = path.join(outputDir, pkgName);
  if (!fs.existsSync(packagePath)) {
    fse.mkdirSync(packagePath);
  }
  return new Promise((resolve, reject) => {
    const args = [idlFilePath, '-o', packagePath];
    const convertor = path.join(__dirname, 'idl_convertor.py');
    const [pythonExecutableFile, pythonExecutableArgs] = pythonExecutable;

    execFile(
      pythonExecutableFile,
      [convertor, ...args],
      (error, stdout, stderr) => {
        if (error) {
          return reject(error);
        }
        if (stderr) {
          console.error(stderr);
        }
        resolve();
      }
    );
  });
}

module.exports = convertIDLToROS2IDL;
