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

const fs = require('mz/fs');
const mkdirp = require('mkdirp');
const path = require('path');
const dot = require('dot');
const requireFromString = require('require-from-string');

const primitiveTypes = require('../rosidl_gen/generator_primitive.js');
const parser = require('../rosidl_parser/rosidl_parser.js');

dot.templateSettings.strip = false;
dot.log = process.env.RCLNODEJS_LOG_VERBOSE || false;
const dots = dot.process({path: path.join(__dirname, '../rosidl_gen/templates')});

const removeExtraSpaceLines = function(str) {
  return str.replace(/([ \t]*\n){2,}/g, '\n');
};

function generateMessageSourceCode(messageType, spec) {
  const className = getClassName(messageType, spec);
  const generated = removeExtraSpaceLines(dots.message({
    className: className,
    msgType: messageType,
    spec: spec,
    types: primitiveTypes,
    json: JSON.stringify(spec, null, '  '),
  }));
  return generated;
}

function getClassName(msgType, spec) {
  return spec.baseType.pkgName + '__' + msgType.msgSubfolder + '__' + spec.msgName;
}

function __discardGenerateMessageStructure(messageType, spec) {
  const className = getClassName(messageType, spec);
  const generated = removeExtraSpaceLines(dots.message({
    className: className,
    msgType: messageType,
    spec: spec,
    types: primitiveTypes,
    json: JSON.stringify(spec, null, '  '),
  }));
  const fileFolder = path.join(__dirname, '../generated/');
  mkdirp.sync(fileFolder);
  const fileName = fileFolder + className + '.js';
  fs.writeFile(fileName, generated);  // Write to file for reference
  return generated;
}

function createMessageObjectFromSpec(messageType, spec) {
  const code = __discardGenerateMessageStructure(messageType, spec);
  return requireFromString(code);
}

function getMessageFileName(msgType) {
  return msgType.pkgName + '/' + msgType.msgSubfolder + '/' + msgType.msgName + '.msg';
}

function getMessagePath(basePath, msgType) {
  return basePath +
    msgType.pkgName + '/' + msgType.msgSubfolder + '/' + msgType.msgName + '.msg';
}

const generatedFilesDir = path.join(__dirname, '../generated/');
mkdirp.sync(generatedFilesDir);

const message = {
  calcMessageFileName: function(messageType) {
    return getMessageFileName(messageType);
  },

  getMessageType: function(pkgName, msgSubfolder, msgName) {
    return {
      pkgName: pkgName,
      msgSubfolder: msgSubfolder,
      msgName: msgName.replace(/\.msg$/g, '')
    };
  },

  generateMessage: function(basePath, messageType) {
    const packagePath = getMessagePath(basePath, messageType);

    return new Promise(function(resolve, reject) {
      parser.parseMessageFile(messageType.pkgName, packagePath).then((spec) => {
        const code = generateMessageSourceCode(messageType, spec);
        const className = getClassName(messageType, spec);
        const fileName = generatedFilesDir + className + '.js';
        return fs.writeFile(fileName, code);
      }).then(() => {
        resolve();
      }).catch((e) => {
        reject(e);
      });
    });  // new Promise
  },

  getMessage: function(basePath, messageType) {
    const packagePath = getMessagePath(basePath, messageType);

    return new Promise(function(resolve, reject) {
      parser.parseMessageFile(messageType.pkgName, packagePath).then((spec) => {
        resolve(createMessageObjectFromSpec(messageType, spec));
      }).catch((e) => {
        reject(e);
      });
    });  // new Promise
  },
};

module.exports = message;
