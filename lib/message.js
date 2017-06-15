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

const fs = require('fs.promised');
const mkdirp = require('mkdirp');
const path = require('path');
const dot = require('dot');
const requireFromString = require('require-from-string');

const primitiveTypes = require('../rosidl_parser/generator_primitive.js');
const Parser = require('../rosidl_parser/rosidl_parser.js');

dot.templateSettings.strip = false;
dot.log = process.env.RCLNODEJS_LOG_VERBOSE || false;
const dots = dot.process({path: path.join(__dirname, '../rosidl_parser/templates')});

const removeExtraSpaceLines = function(str) {
  /* eslint-disable indent */
  return str.replace(/([ \t]*\n){2,}/g, '\n\n') // multiple blank lines into one
            .replace(/\n\n};/g, '\n};')         // remove blank line before };
            .replace(/{\n\n/g, '{\n');          // remove blank line after {
  /* eslint-enable indent */
};

function generateMessageStructure(messageType, spec) {
  const className = spec.baseType.pkgName + '__' + messageType.msgSubfolder + '__' + spec.msgName;
  const generated = removeExtraSpaceLines(dots.message({
    className: className,
    msgType: messageType,
    spec: spec,
    types: primitiveTypes,
  }));
  const fileFolder = path.join(__dirname, '../generated/');
  mkdirp.sync(fileFolder);
  const fileName = fileFolder + className + '.js';
  fs.writeFile(fileName, generated);  // Write to file for reference
  return generated;
}

function createMessageObjectFromSpec(messageType, spec) {
  const code = generateMessageStructure(messageType, spec);
  return requireFromString(code);
}

const message = {
  createMessage: function(messageType) {
    const rosInstallPath = process.env.AMENT_PREFIX_PATH;
    const packagePath = rosInstallPath + '/share/' +
      messageType.pkgName + '/' + messageType.msgSubfolder + '/' + messageType.msgName + '.msg';

    return new Promise(function(resolve, reject) {
      Parser.parseMessageFile(messageType.pkgName, packagePath).then((spec) => {
        resolve(createMessageObjectFromSpec(messageType, spec));
      }).catch((e) => {
        reject(e);
      });
    });  // new Promise
  },
};

module.exports = message;
