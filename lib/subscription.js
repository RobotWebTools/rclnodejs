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
const Entity = require('./entity.js');
const debug = require('debug')('rclnodejs:subscription');

/**
 * @class - Class representing a ROS 2 Subscription
 * @hideconstructor
 * Includes support for content-filtering topics beginning with the
 * ROS Humble release. To learn more about content-filtering
 * @see {@link Node#options}
 * @see {@link Node#createSubscription}
 * @see {@link https://www.omg.org/spec/DDS/1.4/PDF|DDS 1.4 specification, Annex B}
 */

class Subscription extends Entity {
  constructor(handle, typeClass, topic, options, callback) {
    super(handle, typeClass, options);
    this._topic = topic;
    this._callback = callback;
    this._isRaw = options.isRaw || false;
  }

  processResponse(msg) {
    debug(`Message of topic ${this._topic} received.`);
    if (this._isRaw) {
      this._callback(msg);
    } else {
      this._callback(msg.toPlainObject(this.typedArrayEnabled));
    }
  }

  static createSubscription(nodeHandle, typeClass, topic, options, callback) {
    let type = typeClass.type();

    // convert contentFilter.parameters to a string[]
    if (options.contentFilter && options.contentFilter.parameters) {
      options.contentFilter.parameters = options.contentFilter.parameters.map(
        (param) => param.toString()
      );
    }

    let handle = rclnodejs.createSubscription(
      nodeHandle,
      type.pkgName,
      type.subFolder,
      type.interfaceName,
      topic,
      options
    );
    return new Subscription(handle, typeClass, topic, options, callback);
  }

  /**
   * @type {string}
   */
  get topic() {
    return rclnodejs.getSubscriptionTopic(this._handle);
  }

  /**
   * @type {boolean}
   */
  get isRaw() {
    return this._isRaw;
  }

  /**
   * Test if the RMW supports content-filtered topics and that this subscription
   * has an active wellformed content-filter.
   * @returns {boolean} True if content-filtering will be applied; otherwise false.
   */
  hasContentFilter() {
    return rclnodejs.hasContentFilter(this.handle);
  }

  /**
   * If the RMW supports content-filtered topics set this subscription's content-filter.
   * @param {object} contentFilter - The content-filter description.
   * @param {string} contentFilter.expression - Specifies the criteria to select messages of interest.
   *  It is similar to the WHERE part of an SQL clause. Clear the current contentFilter if
   *  the expression is undefined or an empty string.
   * @param {object[]}  [contentFilter.parameters=undefined] - Array of objects that give values to
   *  the ‘parameters’ (i.e., "%n" tokens) in the filter_expression. The number of supplied parameters must
   *  fit with the requested values in the filter_expression (i.e., the number of %n tokens). default: undefined.
   * @returns {boolean} - True if successful; false otherwise
   * @see {@link https://www.omg.org/spec/DDS/1.4/PDF|DDS 1.4 specification, Annex B}
   */
  setContentFilter(contentFilter) {
    return contentFilter?.expression
      ? rclnodejs.setContentFilter(this.handle, contentFilter)
      : this.clearContentFilter();
  }

  /**
   * Clear the current content-filter. No filtering is to be applied.
   * @returns {boolean} - True if successful; false otherwise
   */
  clearContentFilter() {
    return this.hasContentFilter()
      ? rclnodejs.clearContentFilter(this.handle)
      : true;
  }
}

module.exports = Subscription;
