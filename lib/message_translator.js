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

/* eslint-disable max-depth */

const debug = require('debug')('rclnodejs:message_translator');

function isTypedArray(value) {
  return ArrayBuffer.isView(value) && !(value instanceof DataView);
}

function copyMsgObject(msg, obj) {
  if (typeof obj === 'object') {
    for (let i in obj) {
      if (typeof msg[i] !== 'undefined') {
        const type = typeof obj[i];
        if (type === 'string' || type === 'number' || type === 'boolean') {
          // A primitive-type value
          msg[i] = obj[i];
        } else if (Array.isArray(obj[i]) || isTypedArray(obj[i])) {
          // It's an array
          if (typeof obj[i][0] === 'object') {
            // It's an array of objects: converting to ROS message objects

            // 1. Extract the element-type first
            // 2. Build the array by translate every elements
            let msgArray = [];
            obj[i].forEach((o) => {
              msgArray.push(toROSMessage(msg[i].classType.elementType, o));
            });
            // 3. Assign
            msg[i].fill(msgArray);
          } else {
            // It's an array of primitive-type elements
            msg[i] = obj[i];
          }
        } else {
          // Proceed further of this object
          copyMsgObject(msg[i], obj[i]);
        }
      } else {
        // Extra fields in obj (but not in msg)
      }
    } // for
  }
}

function verifyMessage(message, obj) {
  if (message.isROSArray) {
    // It's a ROS message array
    //  Note: there won't be any JavaScript array in message

    if (!Array.isArray(obj)) {
      return false;
    }
    // TODO(Kenny): deal with TypedArray in the future
    // TODO(Kenny): if the elements are objects, check the objects
  } else {
    // It's a ROS message
    const def = message.classType.ROSMessageDef;
    let obj = {};
    for (let i in def.fields) {
      const name = def.fields[i].name;
      if (def.fields[i].type.isPrimitiveType) {
        // check type/existence
        switch (def.fields[i].type) {
        case 'char':
        case 'int16':
        case 'int32':
        case 'int64':
        case 'byte':
        case 'uint16':
        case 'uint32':
        case 'uint64':
        case 'float32':
        case 'float64':
          if (typeof obj[name] != 'number') {
            return false;
          }
          break;
        case 'bool':
          if (typeof obj[name] != 'boolean') {
            return false;
          }
          break;
        case 'string':
          if (typeof obj[name] != 'string') {
            return false;
          }
          break;
        }
      } else if (!verifyMessage(message[name], obj[name])) {
        // Proceed further on this member
        return false;
      }
    }
  }
  return true;
}

function verifyMessageStruct(MessageType, obj) {
  const msg = new MessageType();
  return verifyMessage(msg, obj);
}

function toPlainObject(message) {
  if (!message) return undefined;

  // TODO(Kenny): make sure `message` is a ROS message

  if (message.isROSArray) {
    // It's a ROS message array
    //  Note: there won't be any JavaScript array in message
    let array = [];
    message.data.forEach((e) => {
      array.push(toPlainObject(e));  // Translate every elements
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
        //  Note: TypedArray also falls into this branch
        // TODO(Kenny): make sure Int64 & Uint64 type can be copied here
        obj[name] = message[name];
      } else {
        // Proceed further
        obj[name] = toPlainObject(message[name]);
      }
    }
    return obj;
  }
}

function toROSMessage(TypeClass, obj) {
  let msg = new TypeClass();
  const type = typeof obj;
  if (type === 'string' || type === 'number' || type === 'boolean') {
    msg.data = obj;
  } else if (type === 'object') {
    copyMsgObject(msg, obj);
  }
  return msg;
}

module.exports = {
  verifyMessageStruct: verifyMessageStruct,
  toROSMessage: toROSMessage,
  toPlainObject: toPlainObject
};
