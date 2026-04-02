// Copyright (c) 2026 The Robot Web Tools Contributors. All rights reserved.
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

const QoS = require('./qos.js');
const {
  Parameter,
  ParameterType,
  ParameterDescriptor,
} = require('./parameter.js');

/**
 * Enum of overridable QoS policy kinds.
 * Each value corresponds to a QoS property on the {@link QoS} class.
 * @enum {number}
 */
const QoSPolicyKind = Object.freeze({
  HISTORY: 1,
  DEPTH: 2,
  RELIABILITY: 3,
  DURABILITY: 4,
  LIVELINESS: 5,
  AVOID_ROS_NAMESPACE_CONVENTIONS: 6,
});

// Maps QoSPolicyKind -> { qosProp, paramKey, paramType, toParam, fromParam }
const POLICY_MAP = {
  [QoSPolicyKind.HISTORY]: {
    qosProp: 'history',
    paramKey: 'history',
    enumObj: QoS.HistoryPolicy,
    paramType: ParameterType.PARAMETER_STRING,
    toParam: (val, enumObj) => _enumToString(val, enumObj),
    fromParam: (val, enumObj) => _stringToEnum(val, enumObj),
  },
  [QoSPolicyKind.DEPTH]: {
    qosProp: 'depth',
    paramKey: 'depth',
    paramType: ParameterType.PARAMETER_INTEGER,
    toParam: (val) => BigInt(val),
    fromParam: (val) => Number(val),
  },
  [QoSPolicyKind.RELIABILITY]: {
    qosProp: 'reliability',
    paramKey: 'reliability',
    enumObj: QoS.ReliabilityPolicy,
    paramType: ParameterType.PARAMETER_STRING,
    toParam: (val, enumObj) => _enumToString(val, enumObj),
    fromParam: (val, enumObj) => _stringToEnum(val, enumObj),
  },
  [QoSPolicyKind.DURABILITY]: {
    qosProp: 'durability',
    paramKey: 'durability',
    enumObj: QoS.DurabilityPolicy,
    paramType: ParameterType.PARAMETER_STRING,
    toParam: (val, enumObj) => _enumToString(val, enumObj),
    fromParam: (val, enumObj) => _stringToEnum(val, enumObj),
  },
  [QoSPolicyKind.LIVELINESS]: {
    qosProp: 'liveliness',
    paramKey: 'liveliness',
    enumObj: QoS.LivelinessPolicy,
    paramType: ParameterType.PARAMETER_STRING,
    toParam: (val, enumObj) => _enumToString(val, enumObj),
    fromParam: (val, enumObj) => _stringToEnum(val, enumObj),
  },
  [QoSPolicyKind.AVOID_ROS_NAMESPACE_CONVENTIONS]: {
    qosProp: 'avoidRosNameSpaceConventions',
    paramKey: 'avoid_ros_namespace_conventions',
    paramType: ParameterType.PARAMETER_BOOL,
    toParam: (val) => val,
    fromParam: (val) => Boolean(val),
  },
};

/**
 * Convert a numeric enum value to a lowercase string name.
 * @param {number} val - enum numeric value
 * @param {Object} enumObj - the enum object (e.g. QoS.HistoryPolicy)
 * @returns {string}
 */
function _enumToString(val, enumObj) {
  for (const [key, v] of Object.entries(enumObj)) {
    if (v === val) {
      // Strip the RMW prefix: "RMW_QOS_POLICY_HISTORY_KEEP_LAST" -> "keep_last"
      const parts = key.split('_');
      // Find the index after the policy name (HISTORY, RELIABILITY, etc.)
      // Pattern: RMW_QOS_POLICY_<POLICY>_<VALUE>
      const policyNames = [
        'HISTORY',
        'RELIABILITY',
        'DURABILITY',
        'LIVELINESS',
      ];
      let valueStart = 4; // default: skip RMW_QOS_POLICY_<X>_
      for (let i = 3; i < parts.length; i++) {
        if (policyNames.includes(parts[i])) {
          valueStart = i + 1;
          break;
        }
      }
      return parts.slice(valueStart).join('_').toLowerCase();
    }
  }
  return String(val);
}

/**
 * Convert a lowercase string back to a numeric enum value.
 * @param {string} str - the string value (e.g. "keep_last")
 * @param {Object} enumObj - the enum object
 * @returns {number}
 */
function _stringToEnum(str, enumObj) {
  const upper = str.toUpperCase();
  for (const [key, val] of Object.entries(enumObj)) {
    if (key.endsWith('_' + upper)) {
      return val;
    }
  }
  throw new Error(`Unknown QoS policy value: "${str}"`);
}

/**
 * Options for overriding QoS policies via ROS parameters.
 *
 * When passed to `createPublisher()` or `createSubscription()`, the node
 * will declare read-only parameters for each specified policy kind. These
 * parameters can be set via command-line arguments, launch files, or
 * parameter files to override the QoS profile at startup.
 *
 * Parameter naming convention:
 *   `qos_overrides.<topic>.<publisher|subscription>[_<entityId>].<policy>`
 *
 * @example
 * // Override history, depth, and reliability via parameters
 * const sub = node.createSubscription(
 *   'std_msgs/msg/String', '/chatter',
 *   { qos: rclnodejs.QoS.profileDefault,
 *     qosOverridingOptions: QoSOverridingOptions.withDefaultPolicies() },
 *   (msg) => console.log(msg.data)
 * );
 * // Now you can override via CLI:
 * //   --ros-args -p "qos_overrides./chatter.subscription.depth:=20"
 */
class QoSOverridingOptions {
  /**
   * @param {Array<QoSPolicyKind>} policyKinds - Which QoS policies to expose as parameters.
   * @param {Object} [opts]
   * @param {function} [opts.callback] - Optional validation callback. Receives the
   *   final QoS profile after overrides are applied. Should return
   *   `{successful: true}` or `{successful: false, reason: '...'}`.
   * @param {string} [opts.entityId] - Optional suffix to disambiguate multiple
   *   publishers/subscriptions on the same topic.
   */
  constructor(policyKinds, opts = {}) {
    this._policyKinds = Array.from(policyKinds);
    this._callback = opts.callback || null;
    this._entityId = opts.entityId || null;
  }

  get policyKinds() {
    return this._policyKinds;
  }

  get callback() {
    return this._callback;
  }

  get entityId() {
    return this._entityId;
  }

  /**
   * Create options that override history, depth, and reliability —
   * the most commonly tuned policies.
   * @param {Object} [opts]
   * @param {function} [opts.callback] - Validation callback.
   * @param {string} [opts.entityId] - Entity disambiguation suffix.
   * @returns {QoSOverridingOptions}
   */
  static withDefaultPolicies(opts = {}) {
    return new QoSOverridingOptions(
      [QoSPolicyKind.HISTORY, QoSPolicyKind.DEPTH, QoSPolicyKind.RELIABILITY],
      opts
    );
  }
}

/**
 * Resolve QoS profile string to a mutable QoS object.
 * If already a QoS instance, return as-is.
 * @param {QoS|string} qos
 * @returns {QoS}
 */
function _resolveQoS(qos) {
  if (qos instanceof QoS) {
    return qos;
  }
  // Plain object with QoS fields — construct a QoS from its properties
  if (typeof qos === 'object' && qos !== null && !Array.isArray(qos)) {
    return new QoS(
      qos.history,
      qos.depth,
      qos.reliability,
      qos.durability,
      qos.liveliness,
      qos.avoidRosNameSpaceConventions
    );
  }
  // Profile string: create a QoS with the corresponding defaults
  // Values must match the rmw_qos_profile_* definitions in rmw/types.h
  switch (qos) {
    case QoS.profileDefault:
      return new QoS(
        QoS.HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        10,
        QoS.ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
        QoS.DurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE
      );
    case QoS.profileSystemDefault:
      return new QoS(
        QoS.HistoryPolicy.RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT,
        0,
        QoS.ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
        QoS.DurabilityPolicy.RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT,
        QoS.LivelinessPolicy.RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT
      );
    case QoS.profileSensorData:
      return new QoS(
        QoS.HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        5,
        QoS.ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
        QoS.DurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE
      );
    case QoS.profileServicesDefault:
      return new QoS(
        QoS.HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        10,
        QoS.ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
        QoS.DurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE
      );
    case QoS.profileParameters:
      return new QoS(
        QoS.HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        1000,
        QoS.ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
        QoS.DurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE
      );
    case QoS.profileParameterEvents:
      return new QoS(
        QoS.HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        1000,
        QoS.ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
        QoS.DurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE
      );
    case QoS.profileActionStatusDefault:
      return new QoS(
        QoS.HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        1,
        QoS.ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
        QoS.DurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
      );
    default:
      return new QoS(
        QoS.HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        10,
        QoS.ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
        QoS.DurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE
      );
  }
}

/**
 * Declare QoS override parameters on the node and apply any overrides
 * to the QoS profile in-place.
 *
 * @param {'publisher'|'subscription'} entityType
 * @param {Node} node
 * @param {string} topic - Fully resolved topic name.
 * @param {QoS} qos - Mutable QoS object (will be modified in-place).
 * @param {QoSOverridingOptions} options
 */
function declareQosParameters(entityType, node, topic, qos, options) {
  if (!options || options.policyKinds.length === 0) {
    return;
  }

  const idSuffix = options.entityId ? `_${options.entityId}` : '';
  const namePrefix = `qos_overrides.${topic}.${entityType}${idSuffix}`;

  for (const policyKind of options.policyKinds) {
    const mapping = POLICY_MAP[policyKind];
    if (!mapping) {
      continue;
    }

    const paramName = `${namePrefix}.${mapping.paramKey}`;
    const currentValue = qos[mapping.qosProp];
    const paramValue = mapping.toParam(currentValue, mapping.enumObj);

    const descriptor = new ParameterDescriptor(
      paramName,
      mapping.paramType,
      `QoS override for ${mapping.qosProp}`,
      true // readOnly
    );

    let param;
    try {
      param = node.declareParameter(
        new Parameter(paramName, mapping.paramType, paramValue),
        descriptor
      );
    } catch (e) {
      // Already declared (e.g. multiple entities on same topic) — reuse
      if (node.hasParameter(paramName)) {
        param = node.getParameter(paramName);
      } else {
        throw e;
      }
    }

    // Apply the (possibly overridden) parameter value back to QoS
    if (param && param.value !== paramValue) {
      qos[mapping.qosProp] = mapping.fromParam(param.value, mapping.enumObj);
    }
  }

  // Run validation callback if provided
  if (options.callback) {
    const result = options.callback(qos);
    if (result && !result.successful) {
      throw new Error(
        `QoS override validation failed: ${result.reason || 'unknown reason'}`
      );
    }
  }
}

module.exports = {
  QoSPolicyKind,
  QoSOverridingOptions,
  declareQosParameters,
  _resolveQoS,
};
