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
const fse = require('fs-extra');
const mkdirp = require('mkdirp');
const path = require('path');
const message = require('../rosidl_gen/message.js');

function isMessageDir(dir) {
  return dir.substring(dir.length - 5) === '_msgs' || dir.substring(dir.length - 18) === 'builtin_interfaces';
}

function isMessageFile(file) {
  return file.substring(file.length - 4) === '.msg';
}

function removeAllIntermediateMessages(genPath) {
  return fse.remove(genPath);
}

function generateAllMessages(basePath) {
  return new Promise((resolve, reject) => {
    fs.readdir(basePath).then((rawDirList) => {
      let msgTypeList = [];

      let waitList = [];
      rawDirList.filter(isMessageDir).forEach((item) => {
        const dir = basePath + item + '/msg';
        waitList.push(fs.exists(dir).then((present) => {
          if (present) {
            return fs.readdir(dir);
          }
          return [];
        }).then((rawFileList) => {
          /* eslint-disable max-nested-callbacks */
          rawFileList.filter(isMessageFile).forEach((name) => {
            const msgType = message.getMessageType(item, 'msg', name);
            msgTypeList.push(msgType);
          });
        }));
      });

      Promise.all(waitList).then(() => {
        let msgWaitList = [];
        msgTypeList.forEach((msgType) => {
          msgWaitList.push(message.generateMessage(basePath, msgType));
        });

        return Promise.all(msgWaitList);
      }).then((e) => {
        resolve(msgTypeList);
      }).catch((e) => {
        reject(e);
      });
    }); // fs.readdir
  });
};

function isString(value) {
  if (Array.isArray(value)) {
    let flag = true;
    value.forEach((i) => {
      flag = flag && isString(i);
    });
    return flag;
  }
  return typeof(value) === 'string';
}

const generator = {
  scanPath: undefined,
  genPath: undefined,

  getMessageType: message.getMessageType,

  getMessageClass: function(msgType) {
    let file = '[wrong or missing file name in getMessageClass]';
    if (typeof msgType === 'object' && isString([msgType.pkgName, msgType.msgSubfolder, msgType.msgName])) {
      file = msgType.pkgName + '__' + msgType.msgSubfolder + '__' + msgType.msgName + '.js';
    } else if (isString([arguments[0], arguments[1], arguments[2]])) {
      file = arguments[0] + '__' + arguments[1] + '__' + arguments[2] + '.js';
    }
    return require(this.genPath + file);
  },

  generateAll: function() {
    if (!this.scanPath) {
      const rosInstallPath = process.env.AMENT_PREFIX_PATH;
      this.scanPath = path.join(rosInstallPath, '/share/');
    }

    if (!this.genPath) {
      this.genPath = path.join(__dirname, '../generated/');
    }

    return removeAllIntermediateMessages(this.genPath).then(() => {
      mkdirp.sync(this.genPath);
      return generateAllMessages(this.scanPath);
    });
  },
};

module.exports = generator;
