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

const loader = require('../interface_loader.js');

const CANCEL_GOAL_SERVICE = 'action_msgs/srv/CancelGoal';
const GOAL_INFO_MESSAGE = 'action_msgs/msg/GoalInfo';
const GOAL_STATUS_MESSAGE = 'action_msgs/msg/GoalStatus';
const UUID_MESSAGE = 'unique_identifier_msgs/msg/UUID';

/**
 * @class - Provides lazy access to common action interface types.
 * @ignore
 */

class ActionInterfaces {
  /**
   * Gets the CancelGoal action service interface
   */
  static get CancelGoal() {
    if (!ActionInterfaces._cancelGoalInterface) {
      ActionInterfaces._cancelGoalInterface = loader.loadInterface(
        CANCEL_GOAL_SERVICE
      );
    }

    return ActionInterfaces._cancelGoalInterface;
  }

  /**
   * Gets the GoalInfo action message interface
   */
  static get GoalInfo() {
    if (!ActionInterfaces._goalInfoInterface) {
      ActionInterfaces._goalInfoInterface = loader.loadInterface(
        GOAL_INFO_MESSAGE
      );
    }

    return ActionInterfaces._goalInfoInterface;
  }

  /**
   * Gets the GoalStatus action message interface
   */
  static get GoalStatus() {
    if (!ActionInterfaces._goalStatusInterface) {
      ActionInterfaces._goalStatusInterface = loader.loadInterface(
        GOAL_STATUS_MESSAGE
      );
    }

    return ActionInterfaces._goalStatusInterface;
  }

  /**
   * Gets the UUID action message interface
   */
  static get UUID() {
    if (!ActionInterfaces._uuidInterface) {
      ActionInterfaces._uuidInterface = loader.loadInterface(UUID_MESSAGE);
    }

    return ActionInterfaces._uuidInterface;
  }
}

module.exports = ActionInterfaces;
