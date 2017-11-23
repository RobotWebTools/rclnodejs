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

/* eslint-disable max-depth */
function copyMsgObject(msg, obj) {
  if (typeof obj === 'object') {
    for (let i in obj) {
      if (typeof msg[i] !== 'undefined') {
        const type = typeof obj[i];
        if (type === 'string' || type === 'number' || type === 'boolean') {
          // A primitive-type value
          msg[i] = obj[i];
        } else if (Array.isArray(obj[i])) {  // TODO(Kenny): deal with TypedArray
          // It's an array
          if (typeof obj[i][0] === 'object') {
            // It's an array of objects: converting to ROS message objects

            // 1. Extract the element-type first
            const t = new MessageTranslator(msg[i].classType.elementType);
            // 2. Build the array by translate every elements
            let msgArray = [];
            obj[i].forEach((o) => {
              msgArray.push(t.toROSMessage(o));
            });
            // 3. Assign
            msg[i].fill(msgArray);
          } else {
            // It's an array of primitive-type elements
            msg[i] = obj[i];
          }
        } else {
          // Proceed further of this object
          copyMsgObject(msg[i], obj[i]);  // TODO(Kenny): deal with TypedArray
        }
      } else {
        // Extra fields in obj (but not in msg)
      }
    } // for
  }
}
/* eslint-enable max-depth */

class MessageTranslator {
  constructor(typeClass) {
    this._typeClass = typeClass;
  }

  static toPlainObject(message) {
    // TODO(Kenny): deal with primitive-type array (convert to TypedArray)

    if (message.isROSArray) {
      // It's a ROS message array
      //  Note: there won't be any JavaScript array in message
      let array = [];
      const data = message.data;
      data.forEach((e) => {
        // Translate every elements
        array.push(MessageTranslator.toPlainObject(e));
      });
      return array;
      // eslint-disable-next-line no-else-return
    } else {
      // It's a ROS message
      const def = message.classType.ROSMessageDef;
      let obj = {};
      for (let i in def.fields) {
        const name = def.fields[i].name;
        if (def.fields[i].type.isPrimitiveType) {
          // Direct assignment
          obj[name] = message[name];
        } else {
          // Proceed further
          obj[name] = MessageTranslator.toPlainObject(message[name]);
        }
      }
      return obj;
    }
  }

  toROSMessage(obj) {
    let msg = new this._typeClass();
    const type = typeof obj;
    if (type === 'string' || type === 'number' || type === 'boolean') {
      msg.data = obj;
    } else if (type === 'object') {
      copyMsgObject(msg, obj);
    }
    return msg;
  }
}

module.exports = {
  MessageTranslator: MessageTranslator
};
