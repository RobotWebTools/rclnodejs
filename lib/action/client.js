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
const ClientGoalHandle = require('./client_goal_handle.js');
const Deferred = require('./deferred.js');
const Entity = require('../entity.js');
const loader = require('../interface_loader.js');
const QoS = require('../qos.js');

/**
 * @class - ROS Action client.
 */

class ActionClient extends Entity {
  /**
   * Creates a new action client.
   * @constructor
   *
   * @param {Node} node - The ROS node to add the action client to.
   * @param {function|string|object} typeClass - Type of the action.
   * @param {string} actionName - Name of the action.
   * @param {object} options - Action client options.
   * @param {boolean} options.enableTypedArray - The topic will use TypedArray if necessary, default: true.
   * @param {object} options.qos - ROS Middleware "quality of service" options.
   * @param {QoS} options.qos.goalServiceQosProfile - Quality of service option for the goal service, default: QoS.profileServicesDefault.
   * @param {QoS} options.qos.resultServiceQosProfile - Quality of service option for the result service, default: QoS.profileServicesDefault.
   * @param {QoS} options.qos.cancelServiceQosProfile - Quality of service option for the cancel service, default: QoS.profileServicesDefault..
   * @param {QoS} options.qos.feedbackSubQosProfile - Quality of service option for the feedback subscription,
   *                                                  default: new QoS(QoS.HistoryPolicy.RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT, 10).
   * @param {QoS} options.qos.statusSubQosProfile - Quality of service option for the status subscription, default: QoS.profileActionStatusDefault.
   */
  constructor(node, typeClass, actionName, options) {
    super(null, null, options);

    this._node = node;
    this._typeClass = loader.loadInterface(typeClass);
    this._actionName = actionName;
    this._feedbackCallbacks = new Map();
    this._sequenceNumberGoalIdMap = new Map();
    this._goalHandles = new Map();
    this._pendingGoalRequests = new Map();
    this._pendingCancelRequests = new Map();
    this._pendingResultRequests = new Map();

    // Setup options defaults
    this._options = this._options || {};
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

    let type = this.typeClass.type();

    this._handle = rclnodejs.actionCreateClient(
      node.handle,
      actionName,
      type.interfaceName,
      type.pkgName,
      this.qos.goalServiceQosProfile,
      this.qos.resultServiceQosProfile,
      this.qos.cancelServiceQosProfile,
      this.qos.feedbackSubQosProfile,
      this.qos.statusSubQosProfile
    );

    node._addActionClient(this);
  }

  processGoalResponse(sequence, response) {
    if (this._sequenceNumberGoalIdMap.has(sequence)) {
      let goalHandle = new ClientGoalHandle(
        this,
        this._sequenceNumberGoalIdMap.get(sequence),
        response
      );

      if (goalHandle.accepted) {
        let uuid = ActionUuid.fromBytes(goalHandle.goalId.uuid).toString();
        if (this._goalHandles.has(uuid)) {
          throw new Error(`Two goals were accepted with the same ID (${uuid})`);
        }

        this._goalHandles.set(uuid, goalHandle);
      }

      this._pendingGoalRequests.get(sequence).setResult(goalHandle);
    } else {
      this._node
        .getLogger()
        .warn(
          'Ignoring unexpected goal response. There may be more than ' +
            `one action server for the action '${this._actionName}'`
        );
    }
  }

  processCancelResponse(sequence, response) {
    if (this._pendingCancelRequests.has(sequence)) {
      this._pendingCancelRequests
        .get(sequence)
        .setResult(response.toPlainObject(this.typedArrayEnabled));
    } else {
      this._node
        .getLogger()
        .warn(
          'Ignoring unexpected cancel response. There may be more than ' +
            `one action server for the action '${this._actionName}'`
        );
    }
  }

  processResultResponse(sequence, response) {
    if (this._pendingResultRequests.has(sequence)) {
      this._pendingResultRequests
        .get(sequence)
        .setResult(response.toPlainObject(this.typedArrayEnabled));
    } else {
      this._node
        .getLogger()
        .warn(
          'Ignoring unexpected result response. There may be more than ' +
            `one action server for the action '${this._actionName}'`
        );
    }
  }

  processFeedbackMessage(message) {
    let uuid = ActionUuid.fromBytes(message.goal_id.uuid).toString();
    if (this._feedbackCallbacks.has(uuid)) {
      this._feedbackCallbacks.get(uuid)(
        message.toPlainObject(this.typedArrayEnabled)
      );
    }
  }

  processStatusMessage(message) {
    // Update the status of all goal handles maintained by this Action Client
    for (const statusMessage of message.status_list.data) {
      let uuid = ActionUuid.fromBytes(
        statusMessage.goal_info.goal_id.uuid
      ).toString();
      let status = statusMessage.status;

      if (!this._goalHandles.has(uuid)) {
        let goalHandle = this._goalHandles.get(uuid);
        if (goalHandle) {
          goalHandle._status = status;

          // Remove done handles from the list
          // eslint-disable-next-line max-depth
          if (
            status === ActionInterfaces.GoalStatus.STATUS_SUCCEEDED ||
            status === ActionInterfaces.GoalStatus.STATUS_CANCELED ||
            status === ActionInterfaces.GoalStatus.STATUS_ABORTED
          ) {
            this._goalHandles.delete(uuid);
          }
        }
      } else {
        this._goalHandles.delete(uuid);
      }
    }
  }

  /**
   * Send a goal and wait for the goal ACK asynchronously.
   *
   * Return a Promise object that is resolved with a ClientGoalHandle when receipt of the goal
   * is acknowledged by an action server, see client state transition https://index.ros.org/doc/ros2/Tutorials/Understanding-ROS2-Actions/
   *
   * @param {object} goal - The goal request.
   * @param {function} feedbackCallback - Callback function for feedback associated with the goal.
   * @param {object} goalUuid - Universally unique identifier for the goal. If None, then a random UUID is generated.
   * @returns {Promise} - A Promise to a goal handle that resolves when the goal request has been accepted or rejected.
   */
  sendGoal(goal, feedbackCallback, goalUuid) {
    let request = new this._typeClass.impl.SendGoalService.Request();
    request['goal_id'] = goalUuid || this._createRandomUuid();
    request.goal = goal;

    let sequenceNumber = rclnodejs.actionSendGoalRequest(
      this.handle,
      request.serialize()
    );

    if (this._pendingGoalRequests.has(sequenceNumber)) {
      throw new Error(
        `Sequence (${sequenceNumber}) conflicts with pending goal request`
      );
    }

    if (feedbackCallback) {
      let uuid = ActionUuid.fromBytes(request.goal_id.uuid).toString();
      this._feedbackCallbacks.set(uuid, feedbackCallback);
    }

    let deferred = new Deferred();
    this._pendingGoalRequests.set(sequenceNumber, deferred);
    this._sequenceNumberGoalIdMap.set(sequenceNumber, request.goal_id);

    deferred.setDoneCallback(() =>
      this._removePendingGoalRequest(sequenceNumber)
    );

    return deferred.promise;
  }

  /**
   * Check if there is an action server ready to process requests from this client.
   * @returns {boolean} True if an action server is ready; otherwise, false.
   */
  isActionServerAvailable() {
    return rclnodejs.actionServerIsAvailable(this._node.handle, this.handle);
  }

  /**
   * Wait until the action server is available or a timeout is reached. This
   * function polls for the server state so it may not return as soon as the
   * server is available.
   * @param {number} timeout The maximum amount of time to wait for, if timeout
   * is `undefined` or `< 0`, this will wait indefinitely.
   * @return {Promise<boolean>} true if the service is available.
   */
  async waitForServer(timeout = undefined) {
    let deadline = Infinity;
    if (timeout !== undefined && timeout >= 0) {
      deadline = Date.now() + timeout;
    }
    let waitMs = 5;
    let serviceAvailable = this.isActionServerAvailable();
    while (!serviceAvailable && Date.now() < deadline) {
      waitMs *= 2;
      waitMs = Math.min(waitMs, 1000);
      if (timeout !== undefined && timeout >= -1) {
        waitMs = Math.min(waitMs, deadline - Date.now());
      }
      await new Promise((resolve) => setTimeout(resolve, waitMs));
      serviceAvailable = this.isActionServerAvailable();
    }
    return serviceAvailable;
  }

  /**
   * Send a cancel request for an active goal and asynchronously get the result.
   * @ignore
   * @param {ClientGoalHandle} goalHandle Handle to the goal to cancel.
   * @returns {Promise} - A Promise that resolves when the cancel request has been processed.
   */
  _cancelGoal(goalHandle) {
    if (!(goalHandle instanceof ClientGoalHandle)) {
      throw new TypeError('Invalid argument, must be type of ClientGoalHandle');
    }

    let request = new ActionInterfaces.CancelGoal.Request();
    request.goal_info['goal_id'] = goalHandle.goalId;
    // Initialize the stamp (otherwise we will get an error when we serialize the message)
    request.goal_info.stamp = {
      sec: 0,
      nanosec: 0,
    };

    let sequenceNumber = rclnodejs.actionSendCancelRequest(
      this.handle,
      request.serialize()
    );
    if (this._pendingCancelRequests.has(sequenceNumber)) {
      throw new Error(
        `Sequence (${sequenceNumber}) conflicts with pending cancel request`
      );
    }

    let deferred = new Deferred();
    this._pendingCancelRequests.set(sequenceNumber, deferred);

    deferred.setDoneCallback(() =>
      this._removePendingCancelRequest(sequenceNumber)
    );

    return deferred.promise;
  }

  /**
   * Get the result of an active goal asynchronously.
   *
   * Return a Promise object that is resolved with result, see client state transition https://index.ros.org/doc/ros2/Tutorials/Understanding-ROS2-Actions/
   *
   * @ignore
   * @param {ClientGoalHandle} goalHandle - Handle to the goal to cancel.
   * @returns {Promise} - A Promise that resolves when the get result request has been processed.
   */
  _getResult(goalHandle) {
    if (!(goalHandle instanceof ClientGoalHandle)) {
      throw new TypeError('Invalid argument, must be type of ClientGoalHandle');
    }

    let request = new this.typeClass.impl.GetResultService.Request();
    request['goal_id'] = goalHandle.goalId;

    let sequenceNumber = rclnodejs.actionSendResultRequest(
      this.handle,
      request.serialize()
    );
    if (this._pendingResultRequests.has(sequenceNumber)) {
      throw new Error(
        `Sequence (${sequenceNumber}) conflicts with pending result request`
      );
    }

    let deferred = new Deferred();
    this._pendingResultRequests.set(sequenceNumber, deferred);

    deferred.setDoneCallback(() =>
      this._removePendingResultRequest(sequenceNumber)
    );

    return deferred.promise;
  }

  /**
   * Creates a new random UUID message.
   * @ignore
   * @returns {object} - The new UUID message.
   */
  _createRandomUuid() {
    let uuid = ActionUuid.random();

    let uuidMsg = new ActionInterfaces.UUID();
    uuidMsg.uuid = uuid.bytes;

    return uuidMsg;
  }

  _removePendingGoalRequest(sequenceNumber) {
    this._pendingGoalRequests.delete(sequenceNumber);
    this._sequenceNumberGoalIdMap.delete(sequenceNumber);
  }

  _removePendingResultRequest(sequenceNumber) {
    this._pendingResultRequests.delete(sequenceNumber);
  }

  _removePendingCancelRequest(sequenceNumber) {
    this._pendingCancelRequests.delete(sequenceNumber);
  }

  /**
   * Destroy the underlying action client handle.
   * @return {undefined}
   */
  destroy() {
    if (this._destroyed) {
      return;
    }

    this._goalHandles.clear();

    this._node._destroyEntity(this, this._node._actionClients);
    this._destroyed = true;
  }
}

module.exports = ActionClient;
