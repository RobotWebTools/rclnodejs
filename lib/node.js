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
        subscription.processResponse(subscription.rosMsg);
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

  createPublisher(messageType, topicName) {
    let publisher = Publisher.createPublisher(this._handle,
        messageType, topicName);
    this._publishers.push(publisher);
    return publisher;
  }

  createSubscription(messageType, message, topic, callback) {
    let subscription = Subscription.createSubscription(this, messageType, message, topic,
                                                       callback);
    this._subscriptions.push(subscription);
    return subscription;
  }

  createClient(serviceType, serviceName) {

  }

  createService(serviceType, serviceName, callback) {

  }

  destoryNode() {
    for (let timer in this._timers) {
      rclnodejs.destoryEntity('timer', timer.handle);
    }
    this._timers = [];

    this._publishers.forEach(function(publisher) {
      rclnodejs.destoryEntity('publisher', publisher.handle, this._handle);
    }.bind(this));
    this._publishers = [];

    this._subscriptions = [];
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
