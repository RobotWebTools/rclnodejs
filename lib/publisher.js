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
const debug = require('debug')('rclnodejs:publisher');
const Entity = require('./entity.js');

/**
 * @class - Class representing a Publisher in ROS
 * @hideconstructor
 */

class Publisher extends Entity {
  constructor(handle, typeClass, topic, options) {
    super(handle, typeClass, options);
  }

  /**
   * @type {string}
   */
  get topic() {
    const fullTopic = rclnodejs.getTopic(this._handle); // returns /my_node/mytopic
    return fullTopic.split('/').pop();
  }

  /**
   * Publish a message
   * @param {object|Buffer} message - The message to be sent, could be kind of JavaScript message generated from .msg
   *                                                          or be a Buffer for a raw message.
   * @return {undefined}
   */
  publish(message) {
    if (message instanceof Buffer) {
      rclnodejs.publishRawMessage(this._handle, message);
    } else {
      // Enables call by plain object/number/string argument
      //  e.g. publisher.publish(3.14);
      //       publisher.publish('The quick brown fox...');
      //       publisher.publish({linear: {x: 0, y: 1, z: 2}, ...});
      let messageToSend =
        message instanceof this._typeClass
          ? message
          : new this._typeClass(message);

      let rawMessage = messageToSend.serialize();
      rclnodejs.publish(this._handle, rawMessage);
    }

    debug(`Message of topic ${this.topic} has been published.`);
  }

  static createPublisher(nodeHandle, typeClass, topic, options) {
    let type = typeClass.type();
    let handle = rclnodejs.createPublisher(
      nodeHandle,
      type.pkgName,
      type.subFolder,
      type.interfaceName,
      topic,
      options.qos
    );
    return new Publisher(handle, typeClass, topic, options);
  }
}

module.exports = Publisher;
