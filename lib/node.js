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

  createTimer(period, callback) {
    if (typeof (period) !== 'number' || typeof (callback) !== 'function') {
      throw new TypeError('Invalid argument');
    }

    let timerHandle = rclnodejs.createTimer(period);
    let timer = new Timer(timerHandle, period, callback);
    this._timers.push(timer);

    return timer;
  }

  createPublisher(typeClass, topic) {
    if (typeof (typeClass) !== 'function' || typeof (topic) !== 'string') {
      throw new TypeError('Invalid argument');
    }

    let publisher = Publisher.createPublisher(this._handle, typeClass, topic);
    this._publishers.push(publisher);
    return publisher;
  }

  createSubscription(typeClass, topic, callback) {
    if (typeof (typeClass) !== 'function' || typeof (topic) !== 'string' || typeof (callback) !== 'function') {
      throw new TypeError('Invalid argument');
    }

    let subscription = Subscription.createSubscription(this._handle, typeClass, topic, callback);
    this._subscriptions.push(subscription);
    return subscription;
  }

  createClient(typeClass, serviceName) {
    if (typeof (typeClass) !== 'function' || typeof (serviceName) !== 'string') {
      throw new TypeError('Invalid argument');
    }

    let client = Client.createClient(this._handle, serviceName, typeClass);
    this._clients.push(client);
    return client;
  }

  createService(typeClass, serviceName, callback) {
    if (typeof (typeClass) !== 'function' || typeof (serviceName) !== 'string' || typeof (callback) !== 'function') {
      throw new TypeError('Invalid argument');
    }

    let service = Service.createService(this._handle, serviceName, typeClass, callback);
    this._services.push(service);
    return service;
  }

  destory() {
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

  destoryPublisher(publisher) {

  }

  destorySubscription(subscription) {

  }

  destoryClient(client) {

  }

  destoryService(service) {

  }

  destoryTimer(timer) {

  }
}

module.exports = Node;
