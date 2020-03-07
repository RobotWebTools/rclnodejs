// Copyright (c) 2018 Intel Corporation. All rights reserved.
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

const EventEmitter = require('events');
const time = require('./time_utils.js');
const uuidv4 = require('uuid/v4');

class ActionServerInterface extends EventEmitter {
  constructor(options) {
    super();

    this._rclnodejs = options.rclnodejs;
    this._actionType = options.type;
    this._actionServer = options.actionServer;

    this._node = this._rclnodejs.createNode(`${this._actionServer}_server`);

    this._goalSubscription = this._node.createSubscription(
      this._actionType + 'ActionGoal',
      this._actionServer + '/goal',
      msg => {
        this._handleGoal(msg);
      }
    );

    this._cancelSubscription = this._node.createSubscription(
      'actionlib_msgs/msg/GoalID',
      this._actionServer + '/cancel',
      msg => {
        this._handleCancel(msg);
      }
    );

    this._statusPublisher = this._node.createPublisher(
      'actionlib_msgs/msg/GoalStatusArray',
      this._actionServer + '/status'
    );

    this._feedbackPublisher = this._node.createPublisher(
      this._actionType + 'ActionFeedback',
      this._actionServer + '/feedback'
    );

    this._resultPublisher = this._node.createPublisher(
      this._actionType + 'ActionResult',
      this._actionServer + '/result'
    );
    this._rclnodejs.spin(this._node);
  }

  getType() {
    return this._actionType;
  }

  generateGoalId() {
    return {
      id: uuidv4(),
      stamp: time.now(),
    };
  }

  shutdown() {
    this._node.destroyPublisher(this._statusPublisher);
    this._node.destroyPublisher(this._feedbackPublisher);
    this._node.destroyPublisher(this._resultPublisher);
    this._node.destroySubscription(this._goalSubscription);
    this._node.destroySubscription(this._cancelSubscription);
  }

  _handleGoal(msg) {
    this.emit('goal', msg);
  }

  _handleCancel(msg) {
    this.emit('cancel', msg);
  }

  publishResult(resultMsg) {
    this._resultPublisher.publish(resultMsg);
  }

  publishFeedback(feedbackMsg) {
    this._feedbackPublisher.publish(feedbackMsg);
  }

  publishStatus(statusMsg) {
    this._statusPublisher.publish(statusMsg);
  }
}

module.exports = ActionServerInterface;
