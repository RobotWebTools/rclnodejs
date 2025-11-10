// Copyright (c) 2025, The Robot Web Tools Contributors
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
const DistroUtils = require('./distro.js');
const { OperationError } = require('./errors.js');
const Entity = require('./entity.js');

/**
 * Enumeration for PublisherEventCallbacks event types.
 * @enum {number}
 */
const PublisherEventType = {
  /** @member {number} */
  PUBLISHER_OFFERED_DEADLINE_MISSED: 0,
  /** @member {number} */
  PUBLISHER_LIVELINESS_LOST: 1,
  /** @member {number} */
  PUBLISHER_OFFERED_INCOMPATIBLE_QOS: 2,
  /** @member {number} */
  PUBLISHER_INCOMPATIBLE_TYPE: 3,
  /** @member {number} */
  PUBLISHER_MATCHED: 4,
};

/**
 * Enumeration for SubscriptionEventCallbacks event types.
 * @enum {number}
 */
const SubscriptionEventType = {
  /** @member {number} */
  SUBSCRIPTION_REQUESTED_DEADLINE_MISSED: 0,
  /** @member {number} */
  SUBSCRIPTION_LIVELINESS_CHANGED: 1,
  /** @member {number} */
  SUBSCRIPTION_REQUESTED_INCOMPATIBLE_QOS: 2,
  /** @member {number} */
  SUBSCRIPTION_MESSAGE_LOST: 3,
  /** @member {number} */
  SUBSCRIPTION_INCOMPATIBLE_TYPE: 4,
  /** @member {number} */
  SUBSCRIPTION_MATCHED: 5,
};

class EventHandler extends Entity {
  constructor(handle, callback, eventType, eventTypeName) {
    super(handle, null, null);
    this._callback = callback;
    this._eventType = eventType;
    this._eventTypeName = eventTypeName;
  }

  takeData() {
    const data = rclnodejs.takeEvent(this._handle, {
      [this._eventTypeName]: this._eventType,
    });
    if (this._callback) {
      this._callback(data);
    }
  }
}

/**
 * @class - Class representing a ROS 2 PublisherEventCallbacks
 * @hideconstructor
 */
class PublisherEventCallbacks {
  constructor() {
    if (DistroUtils.getDistroId() < DistroUtils.getDistroId('jazzy')) {
      throw new OperationError(
        'PublisherEventCallbacks is only available in ROS 2 Jazzy and later',
        {
          code: 'UNSUPPORTED_ROS_VERSION',
          entityType: 'publisher event callbacks',
          details: {
            requiredVersion: 'jazzy',
            currentVersion: DistroUtils.getDistroId(),
          },
        }
      );
    }
    this._deadline = null;
    this._incompatible_qos = null;
    this._liveliness = null;
    this._incompatible_type = null;
    this._matched = null;
    this._eventHandlers = [];
  }

  /**
   * Set deadline missed callback.
   * @param {function} callback - The callback function to be called.
   */
  set deadline(callback) {
    this._deadline = callback;
  }

  /**
   * Get deadline missed callback.
   * @return {function} - The callback function.
   */
  get deadline() {
    return this._deadline;
  }

  /**
   * Set incompatible QoS callback.
   * @param {function} callback - The callback function to be called.
   */
  set incompatibleQos(callback) {
    this._incompatible_qos = callback;
  }

  /**
   * Get incompatible QoS callback.
   * @return {function} - The callback function.
   */
  get incompatibleQos() {
    return this._incompatible_qos;
  }

  /**
   * Set liveliness lost callback.
   * @param {function} callback - The callback function to be called.
   */
  set liveliness(callback) {
    this._liveliness = callback;
  }

  /**
   * Get liveliness lost callback.
   * @return {function} - The callback function.
   */
  get liveliness() {
    return this._liveliness;
  }

  /**
   * Set incompatible type callback.
   * @param {function} callback - The callback function to be called.
   */
  set incompatibleType(callback) {
    this._incompatible_type = callback;
  }

  /**
   * Get incompatible type callback.
   * @return {function} - The callback function.
   */
  get incompatibleType() {
    return this._incompatible_type;
  }

  /**
   * Set matched callback.
   * @param {function} callback - The callback function to be called.
   */
  set matched(callback) {
    this._matched = callback;
  }

  /**
   * Get matched callback.
   * @return {function} - The callback function.
   */
  get matched() {
    return this._matched;
  }

  createEventHandlers(publisherHandle) {
    if (this._deadline) {
      const deadlineHandle = rclnodejs.createPublisherEventHandle(
        publisherHandle,
        PublisherEventType.PUBLISHER_OFFERED_DEADLINE_MISSED
      );
      this._eventHandlers.push(
        new EventHandler(
          deadlineHandle,
          this._deadline,
          PublisherEventType.PUBLISHER_OFFERED_DEADLINE_MISSED,
          'publisher_event_type'
        )
      );
    }

    if (this._incompatible_qos) {
      const incompatibleQosHandle = rclnodejs.createPublisherEventHandle(
        publisherHandle,
        PublisherEventType.PUBLISHER_OFFERED_INCOMPATIBLE_QOS
      );
      this._eventHandlers.push(
        new EventHandler(
          incompatibleQosHandle,
          this._incompatible_qos,
          PublisherEventType.PUBLISHER_OFFERED_INCOMPATIBLE_QOS,
          'publisher_event_type'
        )
      );
    }

    if (this._liveliness) {
      const livelinessHandle = rclnodejs.createPublisherEventHandle(
        publisherHandle,
        PublisherEventType.PUBLISHER_LIVELINESS_LOST
      );
      this._eventHandlers.push(
        new EventHandler(
          livelinessHandle,
          this._liveliness,
          PublisherEventType.PUBLISHER_LIVELINESS_LOST,
          'publisher_event_type'
        )
      );
    }

    if (this._incompatible_type) {
      const incompatibleTypeHandle = rclnodejs.createPublisherEventHandle(
        publisherHandle,
        PublisherEventType.PUBLISHER_INCOMPATIBLE_TYPE
      );
      this._eventHandlers.push(
        new EventHandler(
          incompatibleTypeHandle,
          this._incompatible_type,
          PublisherEventType.PUBLISHER_INCOMPATIBLE_TYPE,
          'publisher_event_type'
        )
      );
    }

    if (this._matched) {
      const matchedHandle = rclnodejs.createPublisherEventHandle(
        publisherHandle,
        PublisherEventType.PUBLISHER_MATCHED
      );
      this._eventHandlers.push(
        new EventHandler(
          matchedHandle,
          this._matched,
          PublisherEventType.PUBLISHER_MATCHED,
          'publisher_event_type'
        )
      );
    }

    return this._eventHandlers;
  }

  get eventHandlers() {
    return this._eventHandlers;
  }
}

/**
 * @class - Class representing a ROS 2 SubscriptionEventCallbacks
 * @hideconstructor
 */
class SubscriptionEventCallbacks {
  constructor() {
    if (DistroUtils.getDistroId() < DistroUtils.getDistroId('jazzy')) {
      throw new OperationError(
        'SubscriptionEventCallbacks is only available in ROS 2 Jazzy and later',
        {
          code: 'UNSUPPORTED_ROS_VERSION',
          entityType: 'subscription event callbacks',
          details: {
            requiredVersion: 'jazzy',
            currentVersion: DistroUtils.getDistroId(),
          },
        }
      );
    }
    this._deadline = null;
    this._incompatible_qos = null;
    this._liveliness = null;
    this._message_lost = null;
    this._incompatible_type = null;
    this._matched = null;
    this._eventHandlers = [];
  }

  /**
   * Set the callback for deadline missed event.
   * @param {function} callback - The callback function to be called.
   */
  set deadline(callback) {
    this._deadline = callback;
  }

  /**
   * Get the callback for deadline missed event.
   * @return {function} - The callback function.
   */
  get deadline() {
    return this._deadline;
  }

  /**
   * Set the callback for incompatible QoS event.
   * @param {function} callback - The callback function to be called.
   */
  set incompatibleQos(callback) {
    this._incompatible_qos = callback;
  }

  /**
   * Get the callback for incompatible QoS event.
   * @return {function} - The callback function.
   */
  get incompatibleQos() {
    return this._incompatible_qos;
  }

  /**
   * Set the callback for liveliness changed event.
   * @param {function} callback - The callback function to be called.
   */
  set liveliness(callback) {
    this._liveliness = callback;
  }

  /**
   * Get the callback for liveliness changed event.
   * @return {function} - The callback function.
   */
  get liveliness() {
    return this._liveliness;
  }

  /**
   * Set the callback for message lost event.
   * @param {function} callback - The callback function to be called.
   */
  set messageLost(callback) {
    this._message_lost = callback;
  }

  /**
   * Get the callback for message lost event.
   * @return {function} - The callback function.
   */
  get messageLost() {
    return this._message_lost;
  }

  /**
   * Set the callback for incompatible type event.
   * @param {function} callback - The callback function to be called.
   */
  set incompatibleType(callback) {
    this._incompatible_type = callback;
  }

  /**
   * Get the callback for incompatible type event.
   * @return {function} - The callback function.
   */
  get incompatibleType() {
    return this._incompatible_type;
  }

  /**
   * Set the callback for matched event.
   * @param {function} callback - The callback function to be called.
   */
  set matched(callback) {
    this._matched = callback;
  }

  /**
   * Get the callback for matched event.
   * @return {function} - The callback function.
   */
  get matched() {
    return this._matched;
  }

  createEventHandlers(subscriptionHandle) {
    if (this._deadline) {
      const deadlineHandle = rclnodejs.createSubscriptionEventHandle(
        subscriptionHandle,
        SubscriptionEventType.SUBSCRIPTION_REQUESTED_DEADLINE_MISSED
      );
      this._eventHandlers.push(
        new EventHandler(
          deadlineHandle,
          this._deadline,
          SubscriptionEventType.SUBSCRIPTION_REQUESTED_DEADLINE_MISSED,
          'subscription_event_type'
        )
      );
    }

    if (this._incompatible_qos) {
      const incompatibleQosHandle = rclnodejs.createSubscriptionEventHandle(
        subscriptionHandle,
        SubscriptionEventType.SUBSCRIPTION_REQUESTED_INCOMPATIBLE_QOS
      );
      this._eventHandlers.push(
        new EventHandler(
          incompatibleQosHandle,
          this._incompatible_qos,
          SubscriptionEventType.SUBSCRIPTION_REQUESTED_INCOMPATIBLE_QOS,
          'subscription_event_type'
        )
      );
    }

    if (this._liveliness) {
      const livelinessHandle = rclnodejs.createSubscriptionEventHandle(
        subscriptionHandle,
        SubscriptionEventType.SUBSCRIPTION_LIVELINESS_CHANGED
      );
      this._eventHandlers.push(
        new EventHandler(
          livelinessHandle,
          this._liveliness,
          SubscriptionEventType.SUBSCRIPTION_LIVELINESS_CHANGED,
          'subscription_event_type'
        )
      );
    }

    if (this._message_lost) {
      const messageLostHandle = rclnodejs.createSubscriptionEventHandle(
        subscriptionHandle,
        SubscriptionEventType.SUBSCRIPTION_MESSAGE_LOST
      );
      this._eventHandlers.push(
        new EventHandler(
          messageLostHandle,
          this._message_lost,
          SubscriptionEventType.SUBSCRIPTION_MESSAGE_LOST,
          'subscription_event_type'
        )
      );
    }

    if (this._incompatible_type) {
      const incompatibleTypeHandle = rclnodejs.createSubscriptionEventHandle(
        subscriptionHandle,
        SubscriptionEventType.SUBSCRIPTION_INCOMPATIBLE_TYPE
      );
      this._eventHandlers.push(
        new EventHandler(
          incompatibleTypeHandle,
          this._incompatible_type,
          SubscriptionEventType.SUBSCRIPTION_INCOMPATIBLE_TYPE,
          'subscription_event_type'
        )
      );
    }

    if (this._matched) {
      const matchedHandle = rclnodejs.createSubscriptionEventHandle(
        subscriptionHandle,
        SubscriptionEventType.SUBSCRIPTION_MATCHED
      );
      this._eventHandlers.push(
        new EventHandler(
          matchedHandle,
          this._matched,
          SubscriptionEventType.SUBSCRIPTION_MATCHED,
          'subscription_event_type'
        )
      );
    }

    return this._eventHandlers;
  }
}

module.exports = {
  PublisherEventCallbacks,
  PublisherEventType,
  SubscriptionEventCallbacks,
  SubscriptionEventType,
};
