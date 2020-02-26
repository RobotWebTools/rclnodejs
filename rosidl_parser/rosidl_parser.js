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

const os = require('os');
const exec = require('child_process').exec;
const pyUtils = require('./py_utils');

const pythonExe = pyUtils.getPython('python3');

let rosidlParser = {
  parseMessageFile(packageName, filePath) {
    return this._parseFile('parse_message_file', packageName, filePath);
  },

  parseServiceFile(packageName, filePath) {
    return this._parseFile('parse_service_file', packageName, filePath);
  },

  parseActionFile(packageName, filePath) {
    return this._parseFile('parse_action_file', packageName, filePath);
  },

  _parseFile(...args) {
    return new Promise((resolve, reject) => {
      exec(this._assembleCommand(args), (err, stdout, stderr) => {
        if (err) {
          reject(new Error(stderr));
        } else {
          resolve(JSON.parse(stdout));
        }
      });
    });
  },

  _assembleCommand(args) {
    let command = `${pythonExe} ${__dirname}/parser.py`;
    args.forEach(arg => {
      command += ' ' + arg;
    });
    return command;
  },
};

module.exports = rosidlParser;
