// Copyright (c) 2026, The Robot Web Tools Contributors
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

const { TypeValidationError, OperationError } = require('./errors');
const { normalizeNodeName } = require('./utils');
const validator = require('./validator');
const debug = require('debug')('rclnodejs:parameter_event_handler');

const PARAMETER_EVENT_MSG_TYPE = 'rcl_interfaces/msg/ParameterEvent';
const PARAMETER_EVENT_TOPIC = '/parameter_events';

/**
 * @class ParameterCallbackHandle
 * Opaque handle returned when adding a parameter callback.
 * Used to remove the callback later.
 */
class ParameterCallbackHandle {
  /**
   * @param {string} parameterName - The parameter name
   * @param {string} nodeName - The fully qualified node name
   * @param {Function} callback - The callback function
   * @hideconstructor
   */
  constructor(parameterName, nodeName, callback) {
    this.parameterName = parameterName;
    this.nodeName = nodeName;
    this.callback = callback;
  }
}

/**
 * @class ParameterEventCallbackHandle
 * Opaque handle returned when adding a parameter event callback.
 * Used to remove the callback later.
 */
class ParameterEventCallbackHandle {
  /**
   * @param {Function} callback - The callback function
   * @hideconstructor
   */
  constructor(callback) {
    this.callback = callback;
  }
}

/**
 * @class ParameterEventHandler
 *
 * Monitors and responds to parameter changes on any node in the ROS 2 graph
 * by subscribing to the `/parameter_events` topic.
 *
 * Unlike {@link ParameterWatcher}, which is tied to a single remote node and
 * requires waiting for that node's parameter services, ParameterEventHandler
 * responds to parameter events from any node without needing service availability.
 *
 * Two types of callbacks are supported:
 * - **Parameter callbacks**: fired when a specific parameter on a specific node
 *   is added or changed (new_parameters + changed_parameters).
 *   Note: deleted parameters are not dispatched to parameter callbacks;
 *   use event callbacks to observe deletions.
 * - **Event callbacks**: fired for every ParameterEvent message received,
 *   including deletions.
 *
 * @example
 * const handler = node.createParameterEventHandler();
 *
 * // Watch a specific parameter on a specific node
 * const handle = handler.addParameterCallback(
 *   'my_param',
 *   '/my_node',
 *   (parameter) => {
 *     console.log(`Parameter changed: ${parameter.name} = ${parameter.value}`);
 *   }
 * );
 *
 * // Watch all parameter events
 * const eventHandle = handler.addParameterEventCallback((event) => {
 *   console.log(`Event from node: ${event.node}`);
 * });
 *
 * // Remove callbacks when done
 * handler.removeParameterCallback(handle);
 * handler.removeParameterEventCallback(eventHandle);
 *
 * // Destroy when no longer needed
 * handler.destroy();
 */
class ParameterEventHandler {
  #node;
  #subscription;
  #parameterCallbacks; // Map<string, ParameterCallbackHandle[]> keyed by "paramName\0nodeName"
  #eventCallbacks; // ParameterEventCallbackHandle[]
  #destroyed;

  /**
   * Create a ParameterEventHandler.
   *
   * @param {object} node - The rclnodejs Node used to create the subscription
   * @param {object} [options] - Options
   * @param {object} [options.qos] - QoS profile for the parameter_events subscription
   */
  constructor(node, options = {}) {
    if (!node || typeof node.createSubscription !== 'function') {
      throw new TypeValidationError('node', node, 'Node instance', {
        entityType: 'parameter event handler',
      });
    }

    if (
      options !== undefined &&
      options !== null &&
      typeof options !== 'object'
    ) {
      throw new TypeValidationError('options', options, 'object or undefined', {
        entityType: 'parameter event handler',
      });
    }

    const opts = options || {};

    this.#node = node;
    this.#parameterCallbacks = new Map();
    this.#eventCallbacks = [];
    this.#destroyed = false;

    const subscriptionOptions = opts.qos ? { qos: opts.qos } : undefined;

    this.#subscription = node.createSubscription(
      PARAMETER_EVENT_MSG_TYPE,
      PARAMETER_EVENT_TOPIC,
      subscriptionOptions,
      (event) => this.#handleEvent(event)
    );

    debug('Created ParameterEventHandler on node=%s', node.name());
  }

  /**
   * Add a callback for a specific parameter on a specific node.
   *
   * The callback is invoked whenever the named parameter is added or changed
   * on the specified node. The callback receives the parameter message object
   * (rcl_interfaces/msg/Parameter) with `name` and `value` fields.
   *
   * @param {string} parameterName - Name of the parameter to monitor
   * @param {string} nodeName - Fully qualified name of the node (e.g., '/my_node')
   * @param {Function} callback - Called with (parameter) when the parameter changes
   * @returns {ParameterCallbackHandle} Handle for removing this callback later
   * @throws {Error} If the handler has been destroyed
   * @throws {TypeError} If arguments are invalid
   */
  addParameterCallback(parameterName, nodeName, callback) {
    this.#checkNotDestroyed();

    if (typeof parameterName !== 'string' || parameterName.trim() === '') {
      throw new TypeValidationError(
        'parameterName',
        parameterName,
        'non-empty string',
        { entityType: 'parameter event handler' }
      );
    }

    if (typeof nodeName !== 'string' || nodeName.trim() === '') {
      throw new TypeValidationError('nodeName', nodeName, 'non-empty string', {
        entityType: 'parameter event handler',
      });
    }

    if (typeof callback !== 'function') {
      throw new TypeValidationError('callback', callback, 'function', {
        entityType: 'parameter event handler',
      });
    }

    const resolvedNodeName = normalizeNodeName(nodeName);
    const resolvedParamName = parameterName.trim();
    const handle = new ParameterCallbackHandle(
      resolvedParamName,
      resolvedNodeName,
      callback
    );
    const key = this.#makeKey(resolvedParamName, resolvedNodeName);

    if (!this.#parameterCallbacks.has(key)) {
      this.#parameterCallbacks.set(key, []);
    }

    // Insert at front (FILO order, matching rclpy behavior)
    this.#parameterCallbacks.get(key).unshift(handle);

    debug(
      'Added parameter callback: param=%s node=%s',
      resolvedParamName,
      resolvedNodeName
    );

    return handle;
  }

  /**
   * Configure which node parameter events will be received.
   *
   * If nodeNames is omitted or empty, the current node filter is cleared.
   * When a filter is active, parameter and event callbacks only receive
   * events from the specified nodes.
   *
   * @param {string[]} [nodeNames] - Node names to filter parameter events from.
   *   Relative names are resolved against the handler node namespace.
   * @returns {boolean} True if the filter is active or was successfully cleared.
   */
  configureNodesFilter(nodeNames) {
    this.#checkNotDestroyed();

    if (nodeNames === undefined || nodeNames === null) {
      this.#subscription.clearContentFilter();
      return !this.#subscription.hasContentFilter();
    }

    if (!Array.isArray(nodeNames)) {
      throw new TypeValidationError('nodeNames', nodeNames, 'string[]', {
        entityType: 'parameter event handler',
      });
    }

    if (nodeNames.length === 0) {
      this.#subscription.clearContentFilter();
      return !this.#subscription.hasContentFilter();
    }

    const resolvedNodeNames = nodeNames.map((nodeName, index) => {
      if (typeof nodeName !== 'string' || nodeName.trim() === '') {
        throw new TypeValidationError(
          `nodeNames[${index}]`,
          nodeName,
          'non-empty string',
          {
            entityType: 'parameter event handler',
          }
        );
      }

      const resolvedNodeName = this.#resolvePath(nodeName.trim());
      this.#validateFullyQualifiedNodePath(resolvedNodeName);
      return resolvedNodeName;
    });

    const contentFilter = {
      expression: resolvedNodeNames
        .map((_, index) => `node = %${index}`)
        .join(' OR '),
      parameters: resolvedNodeNames.map((nodeName) => `'${nodeName}'`),
    };

    this.#subscription.setContentFilter(contentFilter);
    return this.#subscription.hasContentFilter();
  }

  /**
   * Remove a previously added parameter callback.
   *
   * @param {ParameterCallbackHandle} handle - The handle returned by addParameterCallback
   * @throws {Error} If the handle is not found or handler is destroyed
   */
  removeParameterCallback(handle) {
    this.#checkNotDestroyed();

    if (!(handle instanceof ParameterCallbackHandle)) {
      throw new TypeValidationError(
        'handle',
        handle,
        'ParameterCallbackHandle',
        { entityType: 'parameter event handler' }
      );
    }

    const key = this.#makeKey(handle.parameterName, handle.nodeName);
    const callbacks = this.#parameterCallbacks.get(key);

    if (!callbacks) {
      throw new OperationError(
        `No callbacks registered for parameter '${handle.parameterName}' on node '${handle.nodeName}'`,
        { entityType: 'parameter event handler' }
      );
    }

    const index = callbacks.indexOf(handle);
    if (index === -1) {
      throw new OperationError("Callback doesn't exist", {
        entityType: 'parameter event handler',
      });
    }

    callbacks.splice(index, 1);

    if (callbacks.length === 0) {
      this.#parameterCallbacks.delete(key);
    }

    debug(
      'Removed parameter callback: param=%s node=%s',
      handle.parameterName,
      handle.nodeName
    );
  }

  /**
   * Add a callback that is invoked for every parameter event.
   *
   * The callback receives the full ParameterEvent message
   * (rcl_interfaces/msg/ParameterEvent) with `node`, `new_parameters`,
   * `changed_parameters`, and `deleted_parameters` fields.
   *
   * @param {Function} callback - Called with (event) for every ParameterEvent
   * @returns {ParameterEventCallbackHandle} Handle for removing this callback later
   * @throws {Error} If the handler has been destroyed
   * @throws {TypeError} If callback is not a function
   */
  addParameterEventCallback(callback) {
    this.#checkNotDestroyed();

    if (typeof callback !== 'function') {
      throw new TypeValidationError('callback', callback, 'function', {
        entityType: 'parameter event handler',
      });
    }

    const handle = new ParameterEventCallbackHandle(callback);

    // Insert at front (FILO order)
    this.#eventCallbacks.unshift(handle);

    debug('Added parameter event callback');

    return handle;
  }

  /**
   * Remove a previously added parameter event callback.
   *
   * @param {ParameterEventCallbackHandle} handle - The handle returned by addParameterEventCallback
   * @throws {Error} If the handle is not found or handler is destroyed
   */
  removeParameterEventCallback(handle) {
    this.#checkNotDestroyed();

    if (!(handle instanceof ParameterEventCallbackHandle)) {
      throw new TypeValidationError(
        'handle',
        handle,
        'ParameterEventCallbackHandle',
        { entityType: 'parameter event handler' }
      );
    }

    const index = this.#eventCallbacks.indexOf(handle);
    if (index === -1) {
      throw new OperationError("Callback doesn't exist", {
        entityType: 'parameter event handler',
      });
    }

    this.#eventCallbacks.splice(index, 1);

    debug('Removed parameter event callback');
  }

  /**
   * Check if the handler has been destroyed.
   *
   * @returns {boolean} True if destroyed
   */
  isDestroyed() {
    return this.#destroyed;
  }

  /**
   * Destroy the handler and clean up resources.
   * Removes the subscription and clears all callbacks.
   */
  destroy() {
    if (this.#destroyed) {
      return;
    }

    debug('Destroying ParameterEventHandler');

    if (this.#subscription) {
      try {
        this.#node.destroySubscription(this.#subscription);
      } catch (error) {
        debug('Error destroying subscription: %s', error.message);
      }
      this.#subscription = null;
    }

    this.#parameterCallbacks.clear();
    this.#eventCallbacks.length = 0;
    this.#destroyed = true;
  }

  /**
   * Get a specific parameter from a ParameterEvent message.
   *
   * @param {object} event - A ParameterEvent message
   * @param {string} parameterName - The parameter name to look for
   * @param {string} nodeName - The node name to match
   * @returns {object|null} The matching parameter message, or null
   * @static
   */
  static getParameterFromEvent(event, parameterName, nodeName) {
    const resolvedNodeName = normalizeNodeName(nodeName);
    const resolvedParamName = (parameterName || '').trim();

    if (normalizeNodeName(event.node) !== resolvedNodeName) {
      return null;
    }

    const allParams = [
      ...(event.new_parameters || []),
      ...(event.changed_parameters || []),
    ];

    for (const param of allParams) {
      if (param.name === resolvedParamName) {
        return param;
      }
    }

    return null;
  }

  /**
   * Get all parameters from a ParameterEvent message (new + changed).
   *
   * @param {object} event - A ParameterEvent message
   * @returns {object[]} Array of parameter messages
   * @static
   */
  static getParametersFromEvent(event) {
    return [
      ...(event.new_parameters || []),
      ...(event.changed_parameters || []),
    ];
  }

  /**
   * Handle incoming parameter event.
   * @private
   */
  #handleEvent(event) {
    const eventNodeName = normalizeNodeName(event.node);

    // Dispatch parameter-specific callbacks by iterating event params
    // and doing direct Map lookups (O(event_params) instead of O(registered_callbacks))
    const allParams = [
      ...(event.new_parameters || []),
      ...(event.changed_parameters || []),
    ];

    for (const parameter of allParams) {
      const key = this.#makeKey(parameter.name, eventNodeName);
      const callbacks = this.#parameterCallbacks.get(key);

      if (callbacks) {
        for (const handle of callbacks.slice()) {
          try {
            handle.callback(parameter);
          } catch (err) {
            debug(
              'Error in parameter callback for %s on %s: %s',
              parameter.name,
              eventNodeName,
              err.message
            );
          }
        }
      }
    }

    // Dispatch event-level callbacks
    for (const handle of this.#eventCallbacks.slice()) {
      try {
        handle.callback(event);
      } catch (err) {
        debug('Error in parameter event callback: %s', err.message);
      }
    }
  }

  /**
   * Create a map key from parameter name and node name.
   * @private
   */
  #makeKey(paramName, nodeName) {
    return `${paramName}\0${nodeName}`;
  }

  /**
   * Resolve a node path to the fully qualified name used in ParameterEvent.node.
   * @private
   */
  #resolvePath(nodePath) {
    // Absolute node paths are already rooted. Relative names are resolved
    // against the handler node namespace before building the content filter.
    const unresolvedPath = nodePath.startsWith('/')
      ? nodePath
      : `${this.#node.namespace().replace(/\/+$/, '')}/${nodePath}`;

    // Collapse repeated separators for inputs like '/ns//node/' or 'nested//node'.
    const resolvedPath = unresolvedPath.replace(/\/+/g, '/');

    // Preserve the root namespace as '/' and strip trailing slashes everywhere
    // else so the filter matches the canonical ParameterEvent.node format.
    if (resolvedPath === '/') {
      return resolvedPath;
    }

    return resolvedPath.replace(/\/+$/, '');
  }

  /**
   * Validate a fully qualified node path before using it in a content filter.
   * @private
   */
  #validateFullyQualifiedNodePath(nodePath) {
    const normalizedPath =
      nodePath.length > 1 ? nodePath.replace(/\/+$/, '') : nodePath;
    const separatorIndex = normalizedPath.lastIndexOf('/');
    const nodeNamespace =
      separatorIndex === 0 ? '/' : normalizedPath.slice(0, separatorIndex);
    const nodeName = normalizedPath.slice(separatorIndex + 1);

    validator.validateNamespace(nodeNamespace);
    validator.validateNodeName(nodeName);
  }

  /**
   * Check if the handler has been destroyed and throw if so.
   * @private
   */
  #checkNotDestroyed() {
    if (this.#destroyed) {
      throw new OperationError('ParameterEventHandler has been destroyed', {
        entityType: 'parameter event handler',
      });
    }
  }
}

module.exports = ParameterEventHandler;
module.exports.ParameterCallbackHandle = ParameterCallbackHandle;
module.exports.ParameterEventCallbackHandle = ParameterEventCallbackHandle;
