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

const Parser = require('../rosidl_parser/rosidl_parser.js');


function createMessageObjectFromSpec(spec) {
  return {
    data: '',
  };
}

const Message = {
  createMessage: function(messageType) {
    const rosInstallPath = process.env.AMENT_PREFIX_PATH;
    // let packagePath = rosInstallPath + '/share/std_msgs/msg/ColorRGBA.msg';
    let packagePath = rosInstallPath + '/share/';
    packagePath += messageType.pkgName;       // 'std_msgs';
    packagePath += '/';
    packagePath += messageType.msgSubfolder;  // 'msg';
    packagePath += '/';
    packagePath += messageType.msgName;       // 'String';
    packagePath += '.msg';

    return new Promise(function(resolve, reject) {
      Parser.parseMessageFile(messageType.pkgName, packagePath).then((spec) => {
        resolve([createMessageObjectFromSpec(spec), spec]);
      }).catch((e) => {
        reject(e);
      });
    });  // new Promise
  },
};

module.exports = Message;
