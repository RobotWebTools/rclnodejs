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
const debug = require('debug')('rcl_publisher');

class Publisher {
  /**
   * Represents a ROS Publisher
   * @constructor
   * @param {RclHandle} handle - A RclHandle object created by createPublisher
   * @param {Object} messageType - the type of the topic, {pkgName: ..., msgSubfolder: ..., msgName: ...}
   * @param {string} topic - the name of the topic
   */
  constructor(handle, messageType, topic) {
    this._handle = handle;
    this._messageType = messageType;
    this._topic = topic;
  }

  get handle() {
    return this._handle;
  }

  get messageType() {
    return this._messageType;
  }

  get topic() {
    return this._topic;
  }

  /**
   * Publish a message
   * @param {Buffer} message - A Node.js buffer object, representing the message to be published.
   * @return {undefined}
   */
  publish(message) {
    let msg;
    if (message instanceof Buffer) {
      msg = message;
    } else if (typeof message.getBuffer === 'function') {
      msg = message.getBuffer();
    }  // TODO(kenny): check messageType and convert raw values to a ros message

    if (msg) {
      rclnodejs.publishMessage(this._handle, msg);
    } else {
      debug('Message was not published:', message);
    }
  }

  static createPublisher(node, messageType, topicName) {
    let handle = rclnodejs.createPublisher(node,
        messageType.pkgName,
        messageType.msgSubfolder,
        messageType.msgName,
        topicName);
    return new Publisher(handle, messageType, topicName);
  }
};

module.exports = Publisher;
