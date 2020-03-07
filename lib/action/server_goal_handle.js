// Copyright (c) 2020 Matt Richard. All rights reserved.
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
const ActionInterfaces = require('./interfaces.js');
const Deferred = require('./deferred.js');
const { GoalEvent } = require('./response.js');

/**
 * @class - Goal handle for working with Action Servers.
 * @hideconstructor
 */

class ServerGoalHandle {
  constructor(actionServer, goalInfo, goalRequest) {
    this._actionServer = actionServer;
    this._goalInfo = goalInfo;
    this._goalRequest = goalRequest;
    this._cancelRequested = false;
    this._deferred = new Deferred();

    this._handle = rclnodejs.actionAcceptNewGoal(
      this._actionServer.handle,
      this._goalInfo.serialize()
    );
  }

  /**
   * Gets the goal request.
   */
  get request() {
    return this._goalRequest;
  }

  /**
   * Gets the goal Id.
   */
  get goalId() {
    return this._goalInfo.goal_id;
  }

  /**
   * Gets if the goal handle is active.
   */
  get isActive() {
    if (this._destroyed) {
      return false;
    }

    return rclnodejs.actionGoalHandleIsActive(this._handle);
  }

  /**
   * Gets if cancellation was requested.
   */
  get isCancelRequested() {
    return this.status === ActionInterfaces.GoalStatus.STATUS_CANCELING;
  }

  /**
   * Gets the status of the goal.
   */
  get status() {
    if (this._destroyed) {
      return ActionInterfaces.GoalStatus.STATUS_UNKNOWN;
    }

    return rclnodejs.actionGoalHandleGetStatus(this._handle);
  }

  /**
   * Updates the goal handle with the execute status and begins exection.
   * @param {function} callback - An optional callback to use instead of the one provided to the action server.
   * @returns {undefined}
   */
  execute(callback) {
    if (!this.isCancelRequested) {
      this._updateState(GoalEvent.EXECUTE);
    }

    this._actionServer.notifyExecute(this, callback);
  }

  /**
   * Sends feedback back to the client.
   * @param {object} feedback - The feedback to send back.
   * @returns {undefined}
   */
  publishFeedback(feedback) {
    // Ignore for already destroyed goal handles
    if (this._destroyed) {
      return;
    }

    let feedbackMessage = new this._actionServer.typeClass.impl.FeedbackMessage();
    feedbackMessage['goal_id'] = this.goalId;
    feedbackMessage.feedback = feedback;

    rclnodejs.actionPublishFeedback(
      this._actionServer.handle,
      feedbackMessage.serialize()
    );
  }

  /**
   * Updates the goal handle with the succeed status.
   * @returns {undefined}
   */
  succeed() {
    this._updateState(GoalEvent.SUCCEED);
  }

  /**
   * Updates the goal handle with the abort status.
   * @returns {undefined}
   */
  abort() {
    this._updateState(GoalEvent.ABORT);
  }

  /**
   * Updates the goal handle with the canceled status.
   * @returns {undefined}
   */
  canceled() {
    this._updateState(GoalEvent.CANCELED);
  }

  /**
   * Marks the goal handle as destroyed. Any further updates to the handle will be ignored.
   * @ignore
   * @returns {undefined}
   */
  destroy() {
    if (this._destroyed) {
      return;
    }

    this._destroyed = true;
  }

  _updateState(event) {
    // Ignore updates for already destroyed goal handles
    if (this._destroyed) {
      return;
    }

    // Update state
    rclnodejs.actionUpdateGoalState(this._handle, event);

    // Publish state change
    rclnodejs.actionPublishStatus(this._actionServer.handle);

    // If it's a terminal state, then also notify the action server
    if (!rclnodejs.actionGoalHandleIsActive(this._handle)) {
      rclnodejs.actionNotifyGoalDone(this._actionServer.handle);
    }
  }
}

module.exports = ServerGoalHandle;
