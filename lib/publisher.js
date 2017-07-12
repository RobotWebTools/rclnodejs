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
   * @param {RclHandle} publisherHandle - A RclHandle object of publisher.
   * @param {RclHandle} nodeHandle - A RclHandle object of node.
   * @param {Object} typeClass - The class of the topic message.
   * @param {string} topic - The name of the topic.
   */
  constructor(publisherHandle, nodeHandle, typeClass, topic) {
    this._handle = publisherHandle;
    this._nodeHandle = nodeHandle;
    this._typeClass = typeClass;
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
   * @param {Object} message - An object of the topic message.
   * @return {void}
   */
  publish(message) {
    // TODO(minggang): Support to convert a plain JavaScript value/object to a ROS message,
    // thus we can invoke this function like: publisher.publish('hello world').
    let rawRosMessage = this._typeClass.getRefBuffer(message);
    if (rawRosMessage) {
      rclnodejs.publishMessage(this._handle, rawRosMessage);
    } else {
      debug('Message was not published:', message);
    }
  }

  static createPublisher(nodeHandle, typeClass, topicName) {
    let type = typeClass.type();
    let handle = rclnodejs.createPublisher(nodeHandle, type.pkgName, type.msgSubfolder, type.msgName, topicName);
    return new Publisher(handle, nodeHandle, typeClass, topicName);
  }
};

module.exports = Publisher;
