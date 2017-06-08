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

class Publisher {
  /**
   * Represents a ROS Publisher
   * @constructor
   * @param {RclHandle} handle - A RclHandle object created by createPublisher
   * @param {buffer} messageType - A Node.js buffer representing the message_type_support object
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

  get topic() {
    return this._topic;
  }

  get messageType() {
    return this._messageType;
  }

  /**
   * Publish a message
   * @param {Buffer} message - A Node.js buffer object, representing the message to be published.
   */
  publish(message) {
    rclnodejs.publishMessage(message);
  }

  /**
   * Publish a string message
   * @param {string} message - A string representing the message to be published.
   */
  publishStringMessage(message) {
    rclnodejs.rcl_publish_std_string_message(this._handle, message);
  }
};

module.exports = Publisher;
