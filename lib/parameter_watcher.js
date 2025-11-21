// Copyright (c) 2025 Mahmoud Alghalayini. All rights reserved.
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

const EventEmitter = require('events');
const { TypeValidationError, OperationError } = require('./errors');
const { normalizeNodeName } = require('./utils');
const debug = require('debug')('rclnodejs:parameter_watcher');

/**
 * @class ParameterWatcher - Watches parameter changes on a remote node
 *
 * Subscribes to /parameter_events and emits 'change' events when
 * watched parameters on the target node are modified.
 *
 * @extends EventEmitter
 */
class ParameterWatcher extends EventEmitter {
  #node;
  #paramClient;
  #subscription;
  #watchedParams;
  #remoteNodeName;
  #destroyed;

  /**
   * Create a ParameterWatcher instance.
   * Note: Use node.createParameterWatcher() instead of calling this directly.
   *
   * @param {object} node - The local rclnodejs Node instance
   * @param {string} remoteNodeName - Name of the remote node to watch
   * @param {string[]} parameterNames - Array of parameter names to watch
   * @param {object} [options] - Options for the parameter client
   * @param {number} [options.timeout=5000] - Default timeout for parameter operations
   * @hideconstructor
   */
  constructor(node, remoteNodeName, parameterNames, options = {}) {
    super();

    if (!node || typeof node.createParameterClient !== 'function') {
      throw new TypeValidationError('node', node, 'Node instance', {
        entityType: 'parameter watcher',
      });
    }

    if (typeof remoteNodeName !== 'string' || remoteNodeName.trim() === '') {
      throw new TypeValidationError(
        'remoteNodeName',
        remoteNodeName,
        'non-empty string',
        {
          entityType: 'parameter watcher',
        }
      );
    }

    if (!Array.isArray(parameterNames) || parameterNames.length === 0) {
      throw new TypeValidationError(
        'parameterNames',
        parameterNames,
        'non-empty array',
        {
          entityType: 'parameter watcher',
        }
      );
    }

    this.#node = node;
    this.#watchedParams = new Set(parameterNames);
    this.#paramClient = node.createParameterClient(remoteNodeName, options);
    // Cache the remote node name for error messages (in case paramClient is destroyed)
    this.#remoteNodeName = this.#paramClient.remoteNodeName;
    this.#subscription = null;
    this.#destroyed = false;

    debug(
      'Created ParameterWatcher for node=%s, params=%o',
      remoteNodeName,
      parameterNames
    );
  }

  /**
   * Get the remote node name being watched.
   * @type {string}
   * @readonly
   */
  get remoteNodeName() {
    return this.#remoteNodeName;
  }

  /**
   * Get the list of watched parameter names.
   * @type {string[]}
   * @readonly
   */
  get watchedParameters() {
    return Array.from(this.#watchedParams);
  }

  /**
   * Start watching for parameter changes.
   * Waits for the remote node's parameter services and subscribes to parameter events.
   *
   * @param {number} [timeout=5000] - Timeout in milliseconds to wait for services
   * @returns {Promise<boolean>} Resolves to true when watching has started
   * @throws {Error} If the watcher has been destroyed
   */
  async start(timeout = 5000) {
    this.#checkNotDestroyed();

    debug('Starting ParameterWatcher for node=%s', this.remoteNodeName);

    const available = await this.#paramClient.waitForService(timeout);

    if (!available) {
      debug(
        'Parameter services not available for node=%s',
        this.remoteNodeName
      );
      return false;
    }

    if (!this.#subscription) {
      this.#subscription = this.#node.createSubscription(
        'rcl_interfaces/msg/ParameterEvent',
        '/parameter_events',
        (event) => this.#handleParameterEvent(event)
      );

      debug('Subscribed to /parameter_events');
    }

    return true;
  }

  /**
   * Get current values of all watched parameters.
   *
   * @param {object} [options] - Options for the parameter client
   * @param {number} [options.timeout] - Timeout in milliseconds
   * @param {AbortSignal} [options.signal] - AbortSignal for cancellation
   * @returns {Promise<Parameter[]>} Array of Parameter objects
   * @throws {Error} If the watcher has been destroyed
   */
  async getCurrentValues(options) {
    this.#checkNotDestroyed();
    return await this.#paramClient.getParameters(
      Array.from(this.#watchedParams),
      options
    );
  }

  /**
   * Add a parameter name to the watch list.
   *
   * @param {string} name - Parameter name to watch
   * @throws {TypeError} If name is not a string
   * @throws {Error} If the watcher has been destroyed
   */
  addParameter(name) {
    this.#checkNotDestroyed();

    if (typeof name !== 'string' || name.trim() === '') {
      throw new TypeValidationError('name', name, 'non-empty string', {
        entityType: 'parameter watcher',
        entityName: this.remoteNodeName,
      });
    }

    const wasAdded = !this.#watchedParams.has(name);
    this.#watchedParams.add(name);

    if (wasAdded) {
      debug('Added parameter to watch list: %s', name);
    }
  }

  /**
   * Remove a parameter name from the watch list.
   *
   * @param {string} name - Parameter name to stop watching
   * @returns {boolean} True if the parameter was in the watch list
   * @throws {Error} If the watcher has been destroyed
   */
  removeParameter(name) {
    this.#checkNotDestroyed();

    const wasRemoved = this.#watchedParams.delete(name);

    if (wasRemoved) {
      debug('Removed parameter from watch list: %s', name);
    }

    return wasRemoved;
  }

  /**
   * Check if the watcher has been destroyed.
   *
   * @returns {boolean} True if destroyed
   */
  isDestroyed() {
    return this.#destroyed;
  }

  /**
   * Destroy the watcher and clean up resources.
   * Unsubscribes from parameter events and destroys the parameter client.
   */
  destroy() {
    if (this.#destroyed) {
      return;
    }

    debug('Destroying ParameterWatcher for node=%s', this.remoteNodeName);

    if (this.#subscription) {
      try {
        this.#node.destroySubscription(this.#subscription);
      } catch (error) {
        debug('Error destroying subscription: %s', error.message);
      }
      this.#subscription = null;
    }

    if (this.#paramClient) {
      try {
        this.#node.destroyParameterClient(this.#paramClient);
      } catch (error) {
        debug('Error destroying parameter client: %s', error.message);
      }
      this.#paramClient = null;
    }

    this.removeAllListeners();

    this.#destroyed = true;
  }

  /**
   * Handle parameter event from /parameter_events topic.
   * @private
   */
  #handleParameterEvent(event) {
    if (normalizeNodeName(event.node) !== this.remoteNodeName) {
      return;
    }

    const relevantChanges = [];

    if (event.new_parameters) {
      const newParams = event.new_parameters.filter((p) =>
        this.#watchedParams.has(p.name)
      );
      relevantChanges.push(...newParams);
    }

    if (event.changed_parameters) {
      const changedParams = event.changed_parameters.filter((p) =>
        this.#watchedParams.has(p.name)
      );
      relevantChanges.push(...changedParams);
    }

    if (event.deleted_parameters) {
      const deletedParams = event.deleted_parameters.filter((p) =>
        this.#watchedParams.has(p.name)
      );
      relevantChanges.push(...deletedParams);
    }

    if (relevantChanges.length > 0) {
      debug(
        'Parameter change detected: %o',
        relevantChanges.map((p) => p.name)
      );
      this.emit('change', relevantChanges);
    }
  }

  /**
   * Check if the watcher has been destroyed and throw if so.
   * @private
   */
  #checkNotDestroyed() {
    if (this.#destroyed) {
      throw new OperationError('ParameterWatcher has been destroyed', {
        code: 'WATCHER_DESTROYED',
        entityType: 'parameter watcher',
        entityName: this.remoteNodeName,
      });
    }
  }
}

module.exports = ParameterWatcher;
