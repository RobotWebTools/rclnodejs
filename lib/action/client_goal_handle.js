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

const ActionInterfaces = require('./interfaces.js');

/**
 * @class - Goal handle for working with Action Clients.
 * @hideconstructor
 */

class ClientGoalHandle {
  constructor(actionClient, goalId, goalResponse) {
    this._actionClient = actionClient;
    this._goalId = goalId;
    this._goalResponse = goalResponse;
    this._status = this.accepted
      ? ActionInterfaces.GoalStatus.STATUS_ACCEPTED
      : ActionInterfaces.GoalStatus.STATUS_UNKNOWN;
  }

  /**
   * Gets the goal Id.
   */
  get goalId() {
    return this._goalId;
  }

  /**
   * Gets the goal response timestamp.
   */
  get stamp() {
    return this._goalResponse.stamp;
  }

  /**
   * Gets if the goal response was accepted.
   * @deprecated use isAccepted()
   */
  get accepted() {
    return this.isAccepted();
  }

  /**
   * Determine if goal is currently executing
   * @returns {bool} - True if goal is executing; otherwise return false.
   */
  isAccepted() {
    return this._goalResponse.accepted;
  }

  /**
   * Determine if goal is currently executing
   * @returns {bool} - True if goal is executing; otherwise return false.
   */
  isExecuting() {
    return this.status === ActionInterfaces.GoalStatus.STATUS_EXECUTING;
  }

  /**
   * Determine if goal is in the process of canceling.
   * @returns {bool} - True if goal is canceling; otherwise return false.
   */
  isCanceling() {
    return this.status === ActionInterfaces.GoalStatus.STATUS_CANCELING;
  }

  /**
   * Determine if goal completed successfullly.
   * @returns {bool} - True if goal completed successfully; otherwise return false.
   */
  isSucceeded() {
    return this.status === ActionInterfaces.GoalStatus.STATUS_SUCCEEDED;
  }

  /**
   * Determine if goal has been canceled.
   * @returns {bool} - True if goal has been aborted; otherwise return false.
   */
  isCanceled() {
    return this.status === ActionInterfaces.GoalStatus.STATUS_CANCELED;
  }

  /**
   * Determine if goal has been aborted.
   * @returns {bool} - True if goal was aborted; otherwise return false.
   */
  isAborted() {
    return this.status === ActionInterfaces.GoalStatus.STATUS_ABORTED;
  }

  /**
   * Gets the goal status.
   */
  get status() {
    return this._status;
  }

  /**
   * Update status to the latest state of goal computation.
   * When status is in a final state it can not be revered to an
   * earlier state, e.g., can not change from SUCCEEDED to ACCEPTED.
   * @param {number} newStatus - The new status of this goal.
   */
  set status(newStatus) {
    if (
      this._status < ActionInterfaces.GoalStatus.STATUS_SUCCEEDED &&
      newStatus > this._status
    ) {
      this._status = newStatus;
    }
  }

  /**
   * Send a cancel request for the goal.
   * @returns {Promise} - The cancel response.
   */
  cancelGoal() {
    return this._actionClient._cancelGoal(this);
  }

  /**
   * Request the result for the goal.
   * @returns {Promise} - The result response.
   */
  getResult() {
    return this._actionClient._getResult(this);
  }
}

module.exports = ClientGoalHandle;
