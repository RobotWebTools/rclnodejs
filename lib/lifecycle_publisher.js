// Copyright (c) 2020 Wayne Parrott. All rights reserved.
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
const Logging = require('./logging.js');
const Publisher = require('./publisher.js');

/**
 * A publisher that sends messages only when activated.
 * This implementation is based on the
 * {@link https://github.com/ros2/rclcpp/blob/master/rclcpp_lifecycle/include/rclcpp_lifecycle/lifecycle_publisher.hpp|rclcpp LifecyclePublisher class}.
 *
 * @hideconstructor
 */
class LifecyclePublisher extends Publisher {
  constructor(handle, typeClass, topic, options) {
    super(handle, typeClass, options);

    this._enabled = false;
    this._loggger = Logging.getLogger('LifecyclePublisher');
  }

  /**
   * Publish a message only when activated; otherwise do nothing (nop);
   *
   * @param {object|Buffer} message - The message to be sent, could be kind of JavaScript message generated from .msg
   *                                                          or be a Buffer for a raw message.
   * @returns {undefined}
   */
  publish(message) {
    if (!this._enabled) {
      this._loggger.warn(
        `Trying to publish message on the topic ${this.topic}, but the publisher is not activated`
      );

      return;
    }

    return super.publish(message);
  }

  /**
   * Enables communications; publish() will now send messages.
   * @returns {unknown} Void return.
   */
  activate() {
    this._enabled = true;
  }

  /**
   * Disable communications; publish() will not send messages.
   * @returns {unknown} Void return.
   */
  deactivate() {
    this._enabled = false;
  }

  /**
   * Determine if communications are enabled, i.e., activated, or
   * disabled, i.e., deactivated.
   * @returns {boolean} True if activated; otherwise false.
   */
  isActivated() {
    return this._enabled;
  }

  /**
   * A lifecycle-node activation notice.
   * @returns {unknown} Void return.
   */
  onActivate() {
    this.activate();
  }

  /**
   * A lifecycle-node deactivation notice.
   * @returns {unknown} Void return.
   */
  onDeactivate() {
    this.deactivate();
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

    return new LifecyclePublisher(handle, typeClass, topic, options);
  }
}

module.exports = LifecyclePublisher;
