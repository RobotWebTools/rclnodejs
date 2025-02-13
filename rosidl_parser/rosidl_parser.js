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

const compareVersions = require('compare-versions');
const path = require('path');
const execFile = require('child_process').execFile;

const pythonExecutable = require('./py_utils').getPythonExecutable('python3');

const contextSupportedVersion = '21.0.0.0';
const currentVersion = process.version;
const isContextSupported = compareVersions.compare(
  currentVersion.substring(1, currentVersion.length),
  contextSupportedVersion,
  '>='
);

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

  _parseJSONObject(str) {
    // For nodejs >= `contextSupportedVersion`, we leverage context parameter to
    // convert unsafe integer to string, otherwise, json-bigint is used.
    // See https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global_Objects/JSON/parse
    if (isContextSupported) {
      return JSON.parse(str, (key, value, context) => {
        if (
          Number.isInteger(value) &&
          !Number.isSafeInteger(Number(context.source))
        ) {
          return context.source;
        }
        return value;
      });
    }
    const JSONbigString = require('json-bigint')({ storeAsString: true });
    return JSONbigString.parse(str);
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
            resolve(this._parseJSONObject(stdout));
          }
        }
      );
    });
  },
};

module.exports = rosidlParser;
