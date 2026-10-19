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

const pythonExecutable = require('./py_utils.cjs').getPythonExecutable();

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

  /**
   * Parse an .idl file directly using rosidl_parser (no .msg conversion needed).
   * Returns an object with { type: 'message'|'service'|'action', spec: ... }
   * where spec matches the same format as parseMessageFile/parseServiceFile/parseActionFile.
   */
  parseIdlFile(filePath) {
    return new Promise((resolve, reject) => {
      const args = [path.join(__dirname, 'idl_parser.py'), filePath];
      const [pythonExecutableFile, pythonExecutableArgs] = pythonExecutable;
      execFile(
        pythonExecutableFile,
        pythonExecutableArgs.concat(args),
        (err, stdout, stderr) => {
          if (err) {
            reject(
              new Error(
                `There was an error parsing IDL file "${filePath}": "${err}"; stderr was: ${stderr}`
              )
            );
          } else {
            try {
              resolve(this._parseJSONObject(stdout));
            } catch (parseError) {
              reject(parseError);
            }
          }
        }
      );
    });
  },

  _parseJSONObject(str) {
    // Python encodes unsafe integers as decimal strings before serialization.
    return JSON.parse(str);
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
            try {
              resolve(this._parseJSONObject(stdout));
            } catch (parseError) {
              reject(parseError);
            }
          }
        }
      );
    });
  },
};

module.exports = rosidlParser;
