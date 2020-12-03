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
const ActionUuid = require('./uuid.js');
const Entity = require('../entity.js');
const { CancelResponse, GoalEvent, GoalResponse } = require('./response.js');
const loader = require('../interface_loader.js');
const QoS = require('../qos.js');
const ServerGoalHandle = require('./server_goal_handle.js');

/**
 * Execute the goal.
 * @param {ServerGoalHandle} goalHandle - The server goal handle.
 * @returns {undefined}
 */
function defaultHandleAcceptedCallback(goalHandle) {
  goalHandle.execute();
}

/**
 * Accept all goals.
 * @returns {number} - Always responds with acceptance.
 */
function defaultGoalCallback() {
  return GoalResponse.ACCEPT;
}

/**
 * No cancellations.
 * @returns {number} - Always responds with rejection
 */
function defaultCancelCallback() {
  return CancelResponse.REJECT;
}

/**
 * @class - ROS Action server.
 */

class ActionServer extends Entity {
  /**
   * Creates a new action server.
   * @constructor
   *
   * @param {Node} node - The ROS node to add the action server to.
   * @param {function|string|object} typeClass - Type of the action.
   * @param {string} actionName - Name of the action. Used as part of the underlying topic and service names.
   * @param {function} executeCallback - Callback function for processing accepted goals.
   * @param {function} goalCallback - Callback function for handling new goal requests.
   * @param {function} handleAcceptedCallback - Callback function for handling newly accepted goals.
   * @param {function} cancelCallback - Callback function for handling cancel requests.
   * @param {object} options - Action server options.
   * @param {number} options.resultTimeout - How long in seconds a result is kept by the server after a goal reaches a terminal state in seconds, default: 900.
   * @param {boolean} options.enableTypedArray - The topic will use TypedArray if necessary, default: true.
   * @param {object} options.qos - ROS Middleware "quality of service" options.
   * @param {QoS} options.qos.goalServiceQosProfile - Quality of service option for the goal service, default: QoS.profileServicesDefault.
   * @param {QoS} options.qos.resultServiceQosProfile - Quality of service option for the result service, default: QoS.profileServicesDefault.
   * @param {QoS} options.qos.cancelServiceQosProfile - Quality of service option for the cancel service, default: QoS.profileServicesDefault..
   * @param {QoS} options.qos.feedbackSubQosProfile - Quality of service option for the feedback subscription,
   *                                                  default: new QoS(QoS.HistoryPolicy.RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT, 10).
   * @param {QoS} options.qos.statusSubQosProfile - Quality of service option for the status subscription, default: QoS.profileActionStatusDefault.
   */
  constructor(
    node,
    typeClass,
    actionName,
    executeCallback,
    goalCallback = defaultGoalCallback,
    handleAcceptedCallback = defaultHandleAcceptedCallback,
    cancelCallback = defaultCancelCallback,
    options
  ) {
    super(null, null, options);

    this._node = node;
    this._typeClass = loader.loadInterface(typeClass);
    this._actionName = actionName;
    this._goalHandles = new Map();

    // Setup options defaults
    this._options = this._options || {};
    this._options.resultTimeout =
      this._options.resultTimeout != null ? this._options.resultTimeout : 900;
    this._options.enableTypedArray = this._options.enableTypedArray !== false;
    this._options.qos = this._options.qos || {};
    this._options.qos.goalServiceQosProfile =
      this._options.qos.goalServiceQosProfile || QoS.profileServicesDefault;
    this._options.qos.resultServiceQosProfile =
      this._options.qos.resultServiceQosProfile || QoS.profileServicesDefault;
    this._options.qos.cancelServiceQosProfile =
      this._options.qos.cancelServiceQosProfile || QoS.profileServicesDefault;
    this._options.qos.feedbackSubQosProfile =
      this._options.qos.feedbackSubQosProfile ||
      new QoS(QoS.HistoryPolicy.RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT, 10);
    this._options.qos.statusSubQosProfile =
      this._options.qos.statusSubQosProfile || QoS.profileActionStatusDefault;

    this.registerExecuteCallback(executeCallback);
    this.registerGoalCallback(goalCallback);
    this.registerHandleAcceptedCallback(handleAcceptedCallback);
    this.registerCancelCallback(cancelCallback);

    let type = this.typeClass.type();

    this._handle = rclnodejs.actionCreateServer(
      node.handle,
      node.getClock().handle,
      actionName,
      type.interfaceName,
      type.pkgName,
      this.qos.goalServiceQosProfile,
      this.qos.resultServiceQosProfile,
      this.qos.cancelServiceQosProfile,
      this.qos.feedbackSubQosProfile,
      this.qos.statusSubQosProfile,
      this.options.resultTimeout
    );

    node._addActionServer(this);
  }

  processGoalRequest(header, request) {
    this._executeGoalRequest(header, request);
  }

  processCancelRequest(header, request) {
    this._executeCancelRequest(header, request);
  }

  processResultRequest(header, request) {
    this._executeGetResultRequest(header, request);
  }

  processGoalExpired(result, count) {
    this._executeExpiredGoals(result, count);
  }

  /**
   * Register a callback for handling newly accepted goals.
   *
   * The provided function is called whenever a new goal has been accepted by this action server.
   * The function should expect an instance of {@link ServerGoalHandle} as an argument,
   * which represents a handle to the goal that was accepted.
   * The goal handle can be used to interact with the goal, e.g. publish feedback,
   * update the status, or execute a deferred goal.
   *
   * There can only be one handle accepted callback per {@link ActionServer},
   * therefore calling this function will replace any previously registered callback.
   *
   * @param {function} handleAcceptedCallback - Callback function, if not provided,
   * then unregisters any previously registered callback.
   * @returns {undefined}
   */
  registerHandleAcceptedCallback(handleAcceptedCallback) {
    this._handleAcceptedCallback =
      handleAcceptedCallback || defaultHandleAcceptedCallback;
  }

  /**
   * Register a callback for handling new goal requests.
   *
   * The purpose of the goal callback is to decide if a new goal should be accepted or rejected.
   * The callback should take the goal request message as a parameter and must return a {@link GoalResponse} value.
   *
   * @param {function} goalCallback - Callback function, if not provided,
   * then unregisters any previously registered callback.
   * @returns {undefined}
   */
  registerGoalCallback(goalCallback) {
    this._goalCallback = goalCallback || defaultGoalCallback;
  }

  /**
   * Register a callback for handling cancel requests.
   *
   * The purpose of the cancel callback is to decide if a request to cancel an on-going
   * (or queued) goal should be accepted or rejected.
   * The callback should take one parameter containing the cancel request and must return a
   * {@link CancelResponse} value.
   *
   * There can only be one cancel callback per {@link ActionServer}, therefore calling this
   * function will replace any previously registered callback.
   * @param {function} cancelCallback - Callback function, if not provided,
   * then unregisters any previously registered callback.
   * @returns {undefined}
   */
  registerCancelCallback(cancelCallback) {
    this._cancelCallback = cancelCallback || defaultCancelCallback;
  }

  /**
   * Register a callback for executing action goals.
   *
   * The purpose of the execute callback is to execute the action goal and return a result when finished.
   * The callback should take one parameter containing goal request and must return a
   * result instance (i.e. `action_type.Result`).
   *
   * There can only be one execute callback per {@link ActionServer}, therefore calling this
   * function will replace any previously registered callback.
   *
   * @param {function} executeCallback - Callback function.
   * @returns {undefined}
   */
  registerExecuteCallback(executeCallback) {
    if (typeof executeCallback !== 'function') {
      throw new TypeError('Invalid argument');
    }

    this._callback = executeCallback;
  }

  notifyExecute(goalHandle, callback) {
    if (!callback && !this._callback) {
      return;
    }

    this._executeGoal(callback || this._callback, goalHandle);
  }

  _sendResultResponse(header, result) {
    rclnodejs.actionSendResultResponse(this.handle, header, result.serialize());
  }

  _executeGoalRequest(header, request) {
    let goalUuid = request.goal_id;
    let goalInfo = new ActionInterfaces.GoalInfo();
    goalInfo['goal_id'] = goalUuid;

    // Initialize the stamp (otherwise we will get an error when we serialize the message)
    goalInfo.stamp = {
      sec: 0,
      nanosec: 0,
    };

    this._node.getLogger().debug(`New goal request with ID: ${goalUuid.uuid}`);

    let goalIdExists = rclnodejs.actionServerGoalExists(
      this._handle,
      goalInfo.serialize()
    );

    let accepted = false;
    if (!goalIdExists) {
      let result = this._goalCallback(
        request.goal.toPlainObject(this.typedArrayEnabled)
      );

      accepted = result === GoalResponse.ACCEPT;
    }

    let goalHandle;
    if (accepted) {
      // Stamp time of acceptance
      const secondsAndNanos = this._node.getClock().now().secondsAndNanoseconds;
      goalInfo.stamp = {
        sec: secondsAndNanos.seconds,
        nanosec: secondsAndNanos.nanoseconds,
      };

      try {
        goalHandle = new ServerGoalHandle(this, goalInfo, request.goal);
      } catch (error) {
        this._node
          .getLogger()
          .error(
            `Failed to accept new goal with ID ${goalUuid.uuid}: ${error}`
          );
        accepted = false;
      }

      if (accepted) {
        let uuid = ActionUuid.fromBytes(goalUuid.uuid).toString();
        this._goalHandles.set(uuid, goalHandle);
      }
    }

    // Send response
    let response = new this.typeClass.impl.SendGoalService.Response();
    response.accepted = accepted;
    response.stamp = goalInfo.stamp;

    rclnodejs.actionSendGoalResponse(
      this._handle,
      header,
      response.serialize()
    );

    if (!accepted) {
      this._node.getLogger().debug(`New goal rejected: ${goalUuid.uuid}`);
      return;
    }

    this._node.getLogger().debug(`New goal accepted: ${goalUuid.uuid}`);

    this._handleAcceptedCallback(goalHandle);
  }

  async _executeCancelRequest(header, request) {
    let Response = this.typeClass.impl.CancelGoal.Response;
    let response = new Response();

    this._node.getLogger().debug(`Cancel request received: ${request}`);

    // Get list of goals that are requested to be canceled
    const msgHandle = rclnodejs.actionProcessCancelRequest(
      this.handle,
      request.serialize(),
      response.toRawROS()
    );
    let cancelResponse = new Response();
    cancelResponse.deserialize(response.refObject);

    let goalsCanceling = [];
    for (let goalInfo of cancelResponse.goals_canceling.data) {
      let uuid = ActionUuid.fromBytes(goalInfo.goal_id.uuid).toString();
      // Possibly the user doesn't care to track the goal handle
      if (this._goalHandles.has(uuid)) {
        let goalHandle = this._goalHandles.get(uuid);
        let result = await this._cancelCallback(goalHandle);

        if (result === CancelResponse.ACCEPT) {
          // Notify goal handle
          goalHandle._updateState(GoalEvent.CANCEL_GOAL);
          goalsCanceling.push(goalInfo);
        }
      }
    }

    cancelResponse['goals_canceling'] = goalsCanceling;
    rclnodejs.actionSendCancelResponse(
      this.handle,
      header,
      cancelResponse.serialize()
    );

    msgHandle.release();
  }

  async _executeGoal(callback, goalHandle) {
    const goalUuid = goalHandle.goalId.uuid;
    this._node.getLogger().debug(`Executing goal with ID ${goalUuid}`);

    let result;
    try {
      result = await callback(goalHandle);
    } catch (error) {
      // Create an empty result so that we can still send a response to the client
      result = new this.typeClass.Result();

      this._node
        .getLogger()
        .error(`Error raised in execute callback: ${error}`);
    }

    // If user did not trigger a terminal state, assume aborted
    if (goalHandle.isActive) {
      this._node
        .getLogger()
        .warn(`Goal state not set, assuming aborted. Goal ID: ${goalUuid}`);
      goalHandle.abort();
    }

    this._node
      .getLogger()
      .debug(
        `Goal with ID ${goalUuid} finished with state ${goalHandle.status}`
      );

    // Set result
    let response = new this.typeClass.impl.GetResultService.Response();
    response.status = goalHandle.status;
    response.result = result;

    goalHandle._deferred.setResult(response);
  }

  _executeGetResultRequest(header, request) {
    let uuid = ActionUuid.fromBytes(request.goal_id.uuid).toString();

    this._node
      .getLogger()
      .debug(
        `Result request received for goal with ID: ${request.goal_id.uuid}`
      );

    // If no goal with the requested ID exists, then return UNKNOWN status
    if (!this._goalHandles.has(uuid)) {
      this._node
        .getLogger()
        .debug(
          `Sending result response for unknown goal ID: ${request.goal_id.uuid}`
        );

      let response = new this.typeClass.impl.GetResultService.Response();
      response.status = ActionInterfaces.GoalStatus.STATUS_UNKNOWN;
      rclnodejs.actionSendResultResponse(
        this.handle,
        header,
        response.serialize()
      );
      return;
    }

    this._goalHandles
      .get(uuid)
      ._deferred.setDoneCallback((result) =>
        this._sendResultResponse(header, result)
      );
  }

  _executeExpiredGoals(result, count) {
    for (let i = 0; i < count; i++) {
      const goal = result.data[i];

      const goalInfo = new ActionInterfaces.GoalInfo();
      goalInfo.deserialize(goal.refObject);

      let uuid = ActionUuid.fromBytes(goalInfo.goal_id.uuid).toString();
      this._goalHandles.delete(uuid);
    }
  }

  /**
   * Destroy the action server and all goals.
   * @return {undefined}
   */
  destroy() {
    if (this._destroyed) {
      return;
    }

    for (let goalHandle of Array.from(this._goalHandles.values())) {
      goalHandle.destroy();
    }

    this._goalHandles.clear();

    this._node._destroyEntity(this, this._node._actionServers);
    this._destroyed = true;
  }
}

module.exports = ActionServer;
