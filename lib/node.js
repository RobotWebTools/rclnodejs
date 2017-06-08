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

class Node {
  init() {
    this._handle = undefined;
    this._publishers = [];
    this._subscribers = [];
    this._clients = [];
    this._services = [];
    this._timers = [];
    this.spinning = false;
  }

  execute() {
    this._timers.forEach(function(timer, index, array) {
      if (timer.isReady()) {
        rclnodejs.callTimer(timer.handle);
        timer.callback();
      }
    });
  }

  get handle() {
    return this._handle;
  }

  createTimer(period, callback) {
    let timerHandle = rclnodejs.createTimer(period);
    let timer = new Timer(timerHandle, period, callback);
    this._timers.push(timer);

    return timer;
  }

  createPublisher(messageType, topic) {
    // Create publisher handle via node handle
    let pubHandle = rclnodejs.createPublisher(this._handle, undefined, topic);
    let publisher = new Publisher(pubHandle, messageType, topic);
    this._publishers.push(publisher);
    return publisher;
  }

  createSubscription(messageType, topic, callback) {

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

    this._publishers.forEach(function (publisher) {
      rclnodejs.destoryEntity('publisher', publisher.handle, this._handle);
    }.bind(this));
    this._publishers = [];
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
