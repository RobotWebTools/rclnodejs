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
    this._status = ActionInterfaces.GoalStatus.STATUS_UNKNOWN;
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
   */
  get accepted() {
    return this._goalResponse.accepted;
  }

  /**
   * Gets the goal status.
   */
  get status() {
    return this._status;
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
