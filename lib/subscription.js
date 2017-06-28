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

const rclnodejs = require('bindings')('rclnodejs');
const generator = require('../rosidl_gen/generator.js');

class Subscription {
  constructor(handle, messageType, message, topic, callback) {
    this._handle = handle;
    this._messageType = messageType,
    this._topic = topic;
    this._callback = callback;
    this._Message = message;
    this._rosMsg = new message();
  }

  get handle() {
    return this._handle;
  }

  get rosMsg() {
    return this._Message.getRefBuffer(this._rosMsg);
  }

  processResponse(response) {
    this._callback(response);
  }

  static createSubscription(node, messageType, message, topic, callback) {
    let handle = rclnodejs.createSubscription(node._handle, messageType.pkgName,
        messageType.msgSubfolder, messageType.msgName, topic);
    return new Subscription(handle, messageType, message, topic, callback);
  }
};

module.exports = Subscription;
