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

/* eslint-disable */

/**
 * Enum for HistoryPolicy
 * @readonly
 * @enum {number}
 */
let HistoryPolicy = {
  /** @member {number} */
  RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT: 0,
  /** @member {number} */
  RMW_QOS_POLICY_HISTORY_KEEP_LAST: 1,
  /** @member {number} */
  RMW_QOS_POLICY_HISTORY_KEEP_ALL: 2,
};

/**
 * Enum for ReliabilityPolicy
 * @readonly
 * @enum {number}
 */
let ReliabilityPolicy = {
  /** @member {number} */
  RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT: 0,
  /** @member {number} */
  RMW_QOS_POLICY_RELIABILITY_RELIABLE: 1,
  /** @member {number} */
  RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT: 2,
};

/**
 * Enum for DurabilityPolicy
 * @readonly
 * @enum {number}
 */
let DurabilityPolicy = {
  /** @member {number} */
  RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT: 0,
  /** @member {number} */
  RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL: 1,
  /** @member {number} */
  RMW_QOS_POLICY_DURABILITY_VOLATILE: 2,
};

/** Class representing middleware quality of service */
class QoS {
  /**
   * Create a QoS.
   * @param {HistoryPolicy} [history=RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT] - The history value.
   * @param {number} [depth=0] - The depth value.
   * @param {ReliabilityPolicy} [reliability=RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT] - The reliability value.
   * @param {DurabilityPolicy} [durability=RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT] - The durability value.
   * @param {boolean} [avoidRosNameSpaceConventions=false] - The avoidRosNameSpaceConventions value.
   */
  constructor(
    history = HistoryPolicy.RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT,
    depth = 0,
    reliability = ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
    durability = DurabilityPolicy.RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT,
    avoidRosNameSpaceConventions = false
  ) {
    this._history = history;
    this._depth = depth;
    this._reliability = reliability;
    this._durability = durability;
    this._avoidRosNameSpaceConventions = avoidRosNameSpaceConventions;
  }

  /**
   * Get HistoryPolicy enum.
   * @name QoS#static get:HistoryPolicy
   * @function
   * @return {HistoryPolicy}
   */
  static get HistoryPolicy() {
    return HistoryPolicy;
  }

  /**
   * Get ReliabilityPolicy enum.
   * @name QoS#static get:ReliabilityPolicy
   * @function
   * @return {ReliabilityPolicy}
   */
  static get ReliabilityPolicy() {
    return ReliabilityPolicy;
  }

  /**
   * Get DurabilityPolicy enum.
   * @name QoS#static get:DurabilityPolicy
   * @function
   * @return {DurabilityPolicy}
   */
  static get DurabilityPolicy() {
    return DurabilityPolicy;
  }

  /**
   * Get the history value.
   * @name QoS#get:history
   * @function
   * @return {number}
   */
  get history() {
    return this._history;
  }

  /**
   * Set the history value.
   * @param {number} history - value to be set.
   * @name QoS#set:history
   * @function
   * @return {undefined}
   */
  set history(history) {
    if (typeof history !== 'number') {
      throw new TypeError('Invalid argument');
    }

    this._history = history;
  }

  /**
   * Get the depth value.
   * @name QoS#get:depth
   * @function
   * @return {number}
   */
  get depth() {
    return this._depth;
  }

  /**
   * Set the depth value.
   * @param {number} depth - value to be set.
   * @name QoS#set:depth
   * @function
   * @return {undefined}
   */
  set depth(depth) {
    if (typeof depth !== 'number') {
      throw new TypeError('Invalid argument');
    }

    this._depth = depth;
  }

  /**
   * Get the reliability value.
   * @name QoS#get:reliability
   * @function
   * @return {number}
   */
  get reliability() {
    return this._reliability;
  }

  /**
   * Set the reliability value.
   * @param {number} reliability - value to be set.
   * @name QoS#set:reliability
   * @function
   * @return {undefined}
   */
  set reliability(reliability) {
    if (typeof reliability !== 'number') {
      throw new TypeError('Invalid argument');
    }

    this._reliability = reliability;
  }

  /**
   * Get the durability value.
   * @name QoS#get:durability
   * @function
   * @return {number}
   */
  get durability() {
    return this._durability;
  }

  /**
   * Set the durability value.
   * @param {number} durability - value to be set.
   * @name QoS#set:durability
   * @function
   * @return {undefined}
   */
  set durability(durability) {
    if (typeof durability !== 'number') {
      throw new TypeError('Invalid argument');
    }

    this._durability = durability;
  }

  /**
   * Get the avoidRosNameSpaceConventions value.
   * @name QoS#get:avoidRosNameSpaceConventions
   * @function
   * @return {boolean}
   */
  get avoidRosNameSpaceConventions() {
    return this._avoidRosNameSpaceConventions;
  }

  /**
   * Set the avoidRosNameSpaceConventions value.
   * @param {boolean} avoidRosNameSpaceConventions - value to be set.
   * @name QoS#set:avoidRosNameSpaceConventions
   * @function
   * @return {undefined}
   */
  set avoidRosNameSpaceConventions(avoidRosNameSpaceConventions) {
    if (typeof avoidRosNameSpaceConventions !== 'boolean') {
      throw new TypeError('Invalid argument');
    }

    this._avoidRosNameSpaceConventions = avoidRosNameSpaceConventions;
  }

  /**
   * Get default profile.
   * @name QoS#static get:profileDefault
   * @function
   * @return {string}
   */
  static get profileDefault() {
    return 'qos_profile_default';
  }

  /**
   * Get default system profile.
   * @name QoS#static get:profileSystemDefault
   * @function
   * @return {string}
   */
  static get profileSystemDefault() {
    return 'qos_profile_system_default';
  }

  /**
   * Get sensor data profile.
   * @name QoS#static get:profileSensorData
   * @function
   * @return {string}
   */
  static get profileSensorData() {
    return 'qos_profile_sensor_data';
  }

  /**
   * Get default services profile.
   * @name QoS#static get:profileServicesDefault
   * @function
   * @return {string}
   */
  static get profileServicesDefault() {
    return 'qos_profile_services_default';
  }

  /**
   * Get parameters profile.
   * @name QoS#static get:profileParameters
   * @function
   * @return {string}
   */
  static get profileParameters() {
    return 'qos_profile_parameters';
  }

  /**
   * Get parameter events profile.
   * @name QoS#static get:profileParameterEvents
   * @function
   * @return {string}
   */
  static get profileParameterEvents() {
    return 'qos_profile_parameter_events';
  }

  /**
   * Get action status profile.
   * @name QoS#static get:profileActionStatusDefault
   * @function
   * @returns {string}
   */
  static get profileActionStatusDefault() {
    return 'qos_profile_action_status_default';
  }
}

module.exports = QoS;
