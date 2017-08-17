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
const Timer = require('./timer.js');
const Publisher = require('./publisher.js');
const Subscription = require('./subscription.js');
const Client = require('./client.js');
const Service = require('./service.js');

/** Class representing a Node in ROS */
class Node {
  init() {
    this._handle = {};
    this._publishers = [];
    this._subscriptions = [];
    this._clients = [];
    this._services = [];
    this._timers = [];
    this.spinning = false;
  }

  execute() {
    this._timers.forEach((timer) => {
      if (timer.isReady()) {
        rclnodejs.callTimer(timer.handle);
        timer.callback();
      }
    });

    this._subscriptions.forEach((subscription) => {
      let success = rclnodejs.rclTake(subscription.handle, subscription.takenMsg);
      if (success) {
        subscription.processResponse();
      }
    });

    this._clients.forEach((client) => {
      let success = rclnodejs.rclTakeResponse(client.handle, client.sequenceNumber, client.takenResponse);
      if (success) {
        client.processResponse();
      }
    });

    this._services.forEach((service) => {
      let header = rclnodejs.rclTakeRequest(service.handle, this._handle, service.takenRequest);
      if (header) {
        service.processRequest(header);
      }
    });
  }

  handle() {
    return this._handle;
  }

  setHandle(handle) {
    this._handle = handle;
  }

  /**
   * Create a timer.
   * @param {number} period - The number representing period.
   * @param {function} callback - The callback to be called when timeout.
   * @return {Timer} - An instance of Timer.
   */
  createTimer(period, callback) {
    if (typeof (period) !== 'number' || typeof (callback) !== 'function') {
      throw new TypeError('Invalid argument');
    }

    let timerHandle = rclnodejs.createTimer(period);
    let timer = new Timer(timerHandle, period, callback);
    this._timers.push(timer);

    return timer;
  }

  /**
   * Create a publisher.
   * @param {function} typeClass - The class representing the message.
   * @param {string} topic - The name of the topic.
   * @return {Publisher} - An instance of Publisher.
   */
  createPublisher(typeClass, topic) {
    if (typeof (typeClass) !== 'function' || typeof (topic) !== 'string') {
      throw new TypeError('Invalid argument');
    }

    let publisher = Publisher.createPublisher(this._handle, typeClass, topic);
    this._publishers.push(publisher);
    return publisher;
  }

  /**
   * This callback is called when a message is published
   * @callback SubscriptionCallback
   * @param {Object} message - The message published
   * @see [Node.createSubscription]{@link Node#createSubscription}
   * @see [Node.createPublisher]{@link Node#createPublisher}
   * @see {@link Publisher}
   * @see {@link Subscription}
   */

  /**
   * Create a subscription.
   * @param {function} typeClass - The class representing the message.
   * @param {string} topic - The name of the topic.
   * @param {SubscriptionCallback} callback - The callback to be call when receiving the topic subscribed.
   * @return {Subscription} - An instance of Subscription.
   * @see {@link SubscriptionCallback}
   */
  createSubscription(typeClass, topic, callback) {
    if (typeof (typeClass) !== 'function' || typeof (topic) !== 'string' || typeof (callback) !== 'function') {
      throw new TypeError('Invalid argument');
    }

    let subscription = Subscription.createSubscription(this._handle, typeClass, topic, callback);
    this._subscriptions.push(subscription);
    return subscription;
  }

  /**
   * Create a Client.
   * @param {function} typeClass - The Class representing the service.
   * @param {string} serviceName - The service name to request.
   * @return {Client} - An instance of Client.
   */
  createClient(typeClass, serviceName) {
    if (typeof (typeClass) !== 'function' || typeof (serviceName) !== 'string') {
      throw new TypeError('Invalid argument');
    }

    let client = Client.createClient(this._handle, serviceName, typeClass);
    this._clients.push(client);
    return client;
  }

  /**
   * This callback is called when a request is sent to service
   * @callback RequestCallback
   * @param {Object} request - The request sent to the service
   * @see [Node.createService]{@link Node#createService}
   * @see [Client.sendRequest]{@link Client#sendRequest}
   * @see {@link Client}
   * @see {@link Service}
   */

  /**
   * Create a Service.
   * @param {function} typeClass - The class representing the service.
   * @param {string} serviceName - The service name to offer.
   * @param {RequestCallback} callback - The callback to be called when receiving request.
   * @return {Service} - An instance of Service.
   * @see {@link RequestCallback}
   */
  createService(typeClass, serviceName, callback) {
    if (typeof (typeClass) !== 'function' || typeof (serviceName) !== 'string' || typeof (callback) !== 'function') {
      throw new TypeError('Invalid argument');
    }

    let service = Service.createService(this._handle, serviceName, typeClass, callback);
    this._services.push(service);
    return service;
  }

  /**
   * Destroy all resource allocated by this node, including
   * <code>Timer</code>s/<code>Publisher</code>s/<code>Subscription</code>s
   * /<code>Client</code>s/<code>Service</code>s
   * @return {undefined}
   */
  destroy() {
    this.stopSpin();

    this._timers.forEach(function(timer) {
      timer._handle.destroy();
    });
    this._timers = [];

    this._publishers.forEach(function(publisher) {
      publisher._handle.destroy();
    });
    this._publishers = [];

    this._subscriptions.forEach(function(subscription) {
      subscription._handle.destroy();
    });
    this._subscriptions = [];

    this._clients.forEach(function(client) {
      client._handle.destroy();
    });
    this._clients = [];

    this._services.forEach(function(service) {
      service._handle.destroy();
    });
    this._services = [];

    this._handle.destroy();
  }

  destroyPublisher(publisher) {

  }

  destroySubscription(subscription) {

  }

  destroyClient(client) {

  }

  destroyService(service) {

  }

  destroyTimer(timer) {

  }
}

module.exports = Node;
