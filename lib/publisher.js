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

const rclnodejs = require('./native_loader.js');
const debug = require('debug')('rclnodejs:publisher');
const Entity = require('./entity.js');

/**
 * @class - Class representing a Publisher in ROS
 * @hideconstructor
 */

class Publisher extends Entity {
  constructor(handle, typeClass, topic, options, node, eventCallbacks) {
    super(handle, typeClass, options);
    this._node = node;
    if (node && eventCallbacks) {
      this._events = eventCallbacks.createEventHandlers(this.handle);
      node._events.push(...this._events);
    }
  }

  /**
   * @type {string}
   */
  get topic() {
    return rclnodejs.getPublisherTopic(this._handle);
  }

  /**
   * Publish a message
   * @param {object|Buffer} message - The message to be sent, could be kind of JavaScript message generated from .msg
   *                                  or be a Buffer for a raw message.
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

  static createPublisher(node, typeClass, topic, options, eventCallbacks) {
    let type = typeClass.type();
    let handle = rclnodejs.createPublisher(
      node.handle,
      type.pkgName,
      type.subFolder,
      type.interfaceName,
      topic,
      options.qos
    );
    return new Publisher(
      handle,
      typeClass,
      topic,
      options,
      node,
      eventCallbacks
    );
  }

  /**
   * Get the number of subscriptions to this publisher.
   * @returns {number} The number of subscriptions
   */
  get subscriptionCount() {
    return rclnodejs.getSubscriptionCount(this._handle);
  }

  /**
   * Wait until all published message data is acknowledged or until the specified timeout elapses
   *
   * If the timeout is negative then this function will block indefinitely until all published
   * message data is acknowledged.
   * If the timeout is 0 then it will check if all published message has been acknowledged without
   * waiting.
   * If the timeout is greater than 0 then it will return after that period of time has elapsed or
   * all published message data is acknowledged.
   *
   * Raises an error if failed, such as the middleware not supporting this feature.
   *
   * @param {timeout} timeout - The duration to wait for all published message data to be acknowledged in nanoseconds.
   * @return {boolean} `true` if all published message data is acknowledged before the timeout, otherwise `false`.
   */
  waitForAllAcked(timeout) {
    return rclnodejs.waitForAllAcked(this._handle, timeout);
  }

  /**
   * Get the event handlers for this publisher.
   * @returns {Array} The array of event handlers for this publisher.
   */
  get events() {
    return this._events;
  }

  /**
   * Set the event handlers for this publisher.
   * @param {Array} events - The array of event handlers to be set for this publisher.
   */
  set events(events) {
    this._events = events;
  }

  /**
   * Get the logger name for this publisher.
   * @returns {string} The logger name for this publisher.
   */
  get loggerName() {
    return rclnodejs.getNodeLoggerName(this._node.handle);
  }
}

module.exports = Publisher;
