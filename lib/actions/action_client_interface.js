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

/* eslint-disable camelcase */

const EventEmitter = require('events');
const time = require('./time_utils.js');
const uuidv4 = require('uuid/v4');

class ActionClientInterface extends EventEmitter {
  constructor(options) {
    super();

    this._rclnodejs = options.rclnodejs;
    this._actionType = options.type;
    this._actionServer = options.actionServer;
    this._node = this._rclnodejs.createNode(`${this._actionServer}_client`);

    this._goalPublisher = this._node.createPublisher(
      this._actionType + 'ActionGoal',
      this._actionServer + '/goal'
    );

    this._cancelPublisher = this._node.createPublisher(
      'actionlib_msgs/msg/GoalID',
      this._actionServer + '/cancel'
    );

    this._statusSubscription = this._node.createSubscription(
      'actionlib_msgs/msg/GoalStatusArray',
      this._actionServer + '/status',
      msg => {
        this._handleStatus(msg);
      }
    );

    this._feedbackSubscription = this._node.createSubscription(
      this._actionType + 'ActionFeedback',
      this._actionServer + '/feedback',
      msg => {
        this._handleFeedback(msg);
      }
    );

    this._resultSubscription = this._node.createSubscription(
      this._actionType + 'ActionResult',
      this._actionServer + '/result',
      msg => {
        this._handleResult(msg);
      }
    );

    this._goals = {};
    this._goalSeqNum = 0;

    this._rclnodejs.spin(this._node);
  }

  _handleStatus(msg) {
    this.emit('status', msg);
  }

  _handleFeedback(msg) {
    const goalId = msg.status.goal_id.id;
    if (this._goals.hasOwnProperty(goalId)) {
      this.emit('feedback', msg.feedback);
    }
  }

  _handleResult(msg) {
    const goalId = msg.status.goal_id.id;
    if (this._goals.hasOwnProperty(goalId)) {
      delete this._goals[goalId];
      this.emit('result', msg.result);
    }
  }

  cancel(goalId) {
    const cancelGoal = {
      stamp: time.now(),
    };
    if (!goalId) {
      this._cancelPublisher.publish(cancelGoal);
    } else if (this._goals.hasOwnProperty(goalId)) {
      cancelGoal.id = goalId;
      this._cancelPublisher.publish(cancelGoal);
    }
  }

  sendGoal(goal) {
    if (!goal.goal_id) {
      goal.goal_id = {
        stamp: time.now(),
        id: this.generateGoalId(),
      };
    }
    if (!goal.header) {
      goal.header = {
        seq: this._goalSeqNum++,
        stamp: goal.goal_id.stamp,
        frame_id: 'auto-generated',
      };
    }

    const goalId = goal.goal_id.id;
    this._goals[goalId] = goal;
    this._goalPublisher.publish(goal);
    return goal;
  }

  generateGoalId() {
    return uuidv4();
  }

  shutdown() {
    this.removeAllListeners();

    this._node.destroyPublisher(this._goalPublisher);
    this._node.destroyPublisher(this._cancelPublisher);
    this._node.destroySubscription(this._statusSubscription);
    this._node.destroySubscription(this._feedbackSubscription);
    this._node.destroySubscription(this._resultSubscription);
  }
}

module.exports = ActionClientInterface;
