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
const QoS = require('./qos.js');

/**
 * @class - Class representing a Node in ROS
 * @hideconstructor
 */

class Node {
  init(name, namespace) {
    this._publishers = [];
    this._subscriptions = [];
    this._clients = [];
    this._services = [];
    this._timers = [];
    this._name = name;

    if (namespace.length === 0) {
      namespace = '/';
    } else if (!namespace.startsWith('/')) {
      namespace = '/' + namespace;
    }
    this._namespace = namespace;
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
      let Message = subscription.typeClass;
      let msg = new Message();
      let success = rclnodejs.rclTake(subscription.handle, msg.toRawROS());
      if (success) {
        subscription.processResponse(msg.refObject);
      }
      Message.destoryRawROS(msg);
    });

    this._clients.forEach((client) => {
      let Response = client.typeClass.Response;
      let response = new Response();
      let success = rclnodejs.rclTakeResponse(client.handle, client.sequenceNumber, response.toRawROS());
      if (success) {
        client.processResponse(response.refObject);
      }
      Response.destoryRawROS(response);
    });

    this._services.forEach((service) => {
      let Request = service.typeClass.Request;
      let request = new Request();
      let header = rclnodejs.rclTakeRequest(service.handle, this.handle, request.toRawROS());
      if (header) {
        service.processRequest(header, request.refObject);
      }
      Request.destoryRawROS(request);
    });
  }

  startSpinning() {
    this.start();
    this.spinning = true;
  }

  stopSpinning() {
    this.stop();
    this.spinning = false;
  }

  /**
   * Create a Timer.
   * @param {number} period - The number representing period in millisecond.
   * @param {function} callback - The callback to be called when timeout.
   * @return {Timer} - An instance of Timer.
   */
  createTimer(period, callback) {
    if (typeof (period) !== 'number' || typeof (callback) !== 'function') {
      throw new TypeError('Invalid argument');
    }

    // The period unit is millisecond in JavaScript side. When being passed to the
    // C++ side, the value will be converted to nanosecond, which goes into a uint64_t
    // with maxmium value of 2^64-1. So the maxmium is UINT64_MAX in ns, that's 0x10c6f7a0b5ed in ms.
    const MAX_TIMER_PERIOD_IN_MILLISECOND = 0x10c6f7a0b5ed;
    if (period > 0x10c6f7a0b5ed || period < 0) {
      throw new RangeError('Parameter must be between ' + 0 + ' and ' + MAX_TIMER_PERIOD_IN_MILLISECOND);
    }

    let timerHandle = rclnodejs.createTimer(period);
    let timer = new Timer(timerHandle, period, callback);
    this._timers.push(timer);

    return timer;
  }

  /**
   * Create a Publisher.
   * @param {function} typeClass - The class representing the message.
   * @param {string} topic - The name of the topic.
   * @param {QoS} qos - Middleware quality of service settings for the publisher.
   * @return {Publisher} - An instance of Publisher.
   */
  createPublisher(typeClass, topic, qos = QoS.profileDefault) {
    if (typeof (typeClass) !== 'function' || typeof (topic) !== 'string' ||
        (typeof (qos) !== 'string' && !qos instanceof QoS)) {
      throw new TypeError('Invalid argument');
    }

    let publisher = Publisher.createPublisher(this.handle, typeClass, topic, qos);
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
   * Create a Subscription.
   * @param {function} typeClass - The class representing the message.
   * @param {string} topic - The name of the topic.
   * @param {SubscriptionCallback} callback - The callback to be call when receiving the topic subscribed.
   * @param {QoS} qos - Middleware quality of service settings for the subscription.
   * @return {Subscription} - An instance of Subscription.
   * @see {@link SubscriptionCallback}
   */
  createSubscription(typeClass, topic, callback, qos = QoS.profileDefault) {
    if (typeof (typeClass) !== 'function' || typeof (topic) !== 'string' || typeof (callback) !== 'function' ||
        (typeof (qos) !== 'string' && !qos instanceof QoS)) {
      throw new TypeError('Invalid argument');
    }

    let subscription = Subscription.createSubscription(this.handle, typeClass, topic, callback, qos);
    this._subscriptions.push(subscription);
    return subscription;
  }

  /**
   * Create a Client.
   * @param {function} typeClass - The Class representing the service.
   * @param {string} serviceName - The service name to request.
   * @param {QoS} qos - Middleware quality of service settings for the client.
   * @return {Client} - An instance of Client.
   */
  createClient(typeClass, serviceName, qos = QoS.profileDefault) {
    if (typeof (typeClass) !== 'function' || typeof (serviceName) !== 'string' ||
        (typeof (qos) !== 'string' && !qos instanceof QoS)) {
      throw new TypeError('Invalid argument');
    }

    let client = Client.createClient(this.handle, serviceName, typeClass, qos);
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
   * @param {QoS} qos - Middleware quality of service settings for the service.
   * @return {Service} - An instance of Service.
   * @see {@link RequestCallback}
   */
  createService(typeClass, serviceName, callback, qos = QoS.profileDefault) {
    if (typeof (typeClass) !== 'function' || typeof (serviceName) !== 'string' || typeof (callback) !== 'function' ||
        (typeof (qos) !== 'string' && !qos instanceof QoS)) {
      throw new TypeError('Invalid argument');
    }

    let service = Service.createService(this.handle, serviceName, typeClass, callback, qos);
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
    if (this.spinning) {
      this.stopSpinning();
    }

    this.handle.release();
    this._timers = [];
    this._publishers = [];
    this._subscriptions = [];
    this._clients = [];
    this._services = [];
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

  /* Get the name of the node.
   * @return {string}
   */
  name() {
    return rclnodejs.getNodeName(this.handle);
  }

  /* Get the namespace of the node.
   * @return {string}
   */
  namespace() {
    return rclnodejs.getNamespace(this.handle);
  }
}

module.exports = Node;
