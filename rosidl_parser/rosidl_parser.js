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

const path = require('path');
const execFile = require('child_process').execFile;

const pythonExecutable = require('./py_utils').getPythonExecutable('python3');

const rosidlParser = {
  parseMessageFile(packageName, filePath) {
    return this._parseFile('parse_message_file', packageName, filePath);
  },

  parseServiceFile(packageName, filePath) {
    return this._parseFile('parse_service_file', packageName, filePath);
  },

  parseActionFile(packageName, filePath) {
    return this._parseFile('parse_action_file', packageName, filePath);
  },

  _parseFile(command, packageName, filePath) {
    return new Promise((resolve, reject) => {
      const args = [
        path.join(__dirname, 'parser.py'),
        command,
        packageName,
        filePath,
      ];
      const [pythonExecutableFile, pythonExecutableArgs] = pythonExecutable;
      execFile(
        pythonExecutableFile,
        pythonExecutableArgs.concat(args),
        (err, stdout, stderr) => {
          if (err) {
            reject(
              new Error(
                `There was an error executing python with arguments "${JSON.stringify(
                  args
                )}": "${err}"; stderr was: ${stderr}`
              )
            );
          } else {
            resolve(JSON.parse(stdout));
          }
        }
      );
    });
  },
};

module.exports = rosidlParser;
