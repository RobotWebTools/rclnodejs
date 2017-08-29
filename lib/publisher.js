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
const Entity = require('./entity.js');

/** Class representing a Publisher in ROS */
class Publisher extends Entity {
  constructor(handle, nodeHandle, typeClass, topic, qos) {
    super(handle, typeClass, qos);
    this._topic = topic;
  }

  /**
   * @type {string}
   */
  get topic() {
    return this._topic;
  }

  /**
   * Publish a message
   * @param {object} message - The message to be sent.
   * @return {undefined}
   */
  publish(message) {
    // TODO(minggang): Support to convert a plain JavaScript value/object to a ROS message,
    // thus we can invoke this function like: publisher.publish('hello world').
    let rawRosMessage = message.toRawROS();
    if (rawRosMessage) {
      rclnodejs.publish(this._handle, rawRosMessage);
    } else {
      debug('Message was not published:', message);
    }
  }

  static createPublisher(nodeHandle, typeClass, topic, qos) {
    let type = typeClass.type();
    let handle = rclnodejs.createPublisher(nodeHandle, type.pkgName, type.subFolder, type.interfaceName, topic, qos);
    return new Publisher(handle, nodeHandle, typeClass, topic, qos);
  }
};

module.exports = Publisher;
