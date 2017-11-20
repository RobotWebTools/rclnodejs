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

const debug = require('debug')('rclnodejs:message-translator');

function copyMsgObject(msg, obj) {
  if (typeof obj === 'object') {
    for (let i in obj) {
      if (typeof msg[i] !== 'undefined') {
        const type = typeof obj[i];
        if (type === 'string' || type === 'number' || type === 'boolean') {
          msg[i] = obj[i];
        } else {
          copyMsgObject(msg[i], obj[i]);
        }
      }
    }
  }
}

class MsgTranslator {
  constructor(typeClass) {
    this._typeClass = typeClass;
  }

  toObject(message) {

  }

  toMessage(obj) {
    let msg = new this._typeClass();
    const type = typeof obj;
    if (type === 'string' || type === 'number' || type === 'boolean') {
      msg.data = obj;
    } else if (type === 'object') {
      copyMsgObject(msg, obj);
    }

    // console.log('');
    // console.log('copying...');
    // console.log(obj);
    // console.log('--------------->');
    // console.log(msg);
    // console.log('');
    return msg;
  }
}

module.exports = {
  MsgTranslator: MsgTranslator
};
