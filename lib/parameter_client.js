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

const {
  Parameter,
  ParameterType,
  parameterTypeFromValue,
} = require('./parameter.js');
const {
  TypeValidationError,
  ParameterNotFoundError,
  OperationError,
} = require('./errors.js');
const validator = require('./validator.js');
const { normalizeNodeName } = require('./utils.js');
const debug = require('debug')('rclnodejs:parameter_client');

/**
 * @class - Class representing a Parameter Client for accessing parameters on remote nodes
 * @hideconstructor
 */
class ParameterClient {
  #node;
  #remoteNodeName;
  #timeout;
  #clients;
  #destroyed;

  /**
   * Create a ParameterClient instance.
   * @param {Node} node - The node to use for creating service clients.
   * @param {string} remoteNodeName - The name of the remote node whose parameters to access.
   * @param {object} [options] - Options for parameter client.
   * @param {number} [options.timeout=5000] - Default timeout in milliseconds for service calls.
   */
  constructor(node, remoteNodeName, options = {}) {
    if (!node) {
      throw new TypeValidationError('node', node, 'Node instance');
    }
    if (!remoteNodeName || typeof remoteNodeName !== 'string') {
      throw new TypeValidationError(
        'remoteNodeName',
        remoteNodeName,
        'non-empty string'
      );
    }

    this.#node = node;
    this.#remoteNodeName = normalizeNodeName(remoteNodeName);
    validator.validateNodeName(this.#remoteNodeName);

    this.#timeout = options.timeout || 5000;
    this.#clients = new Map();
    this.#destroyed = false;

    debug(
      `ParameterClient created for remote node: ${this.#remoteNodeName} with timeout: ${this.#timeout}ms`
    );
  }

  /**
   * Get the remote node name this client is connected to.
   * @return {string} - The remote node name.
   */
  get remoteNodeName() {
    return this.#remoteNodeName;
  }

  /**
   * Get a single parameter from the remote node.
   * @param {string} name - The name of the parameter to retrieve.
   * @param {object} [options] - Options for the service call.
   * @param {number} [options.timeout] - Timeout in milliseconds for this specific call.
   * @param {AbortSignal} [options.signal] - AbortSignal to cancel the request.
   * @return {Promise<Parameter>} - Promise that resolves with the Parameter object.
   * @throws {Error} If the parameter is not found or service call fails.
   */
  async getParameter(name, options = {}) {
    this.#throwErrorIfClientDestroyed();

    const parameters = await this.getParameters([name], options);
    if (parameters.length === 0) {
      throw new ParameterNotFoundError(name, this.#remoteNodeName);
    }

    return parameters[0];
  }

  /**
   * Get multiple parameters from the remote node.
   * @param {string[]} names - Array of parameter names to retrieve.
   * @param {object} [options] - Options for the service call.
   * @param {number} [options.timeout] - Timeout in milliseconds for this specific call.
   * @param {AbortSignal} [options.signal] - AbortSignal to cancel the request.
   * @return {Promise<Parameter[]>} - Promise that resolves with an array of Parameter objects.
   * @throws {Error} If the service call fails.
   */
  async getParameters(names, options = {}) {
    this.#throwErrorIfClientDestroyed();

    if (!Array.isArray(names) || names.length === 0) {
      throw new TypeValidationError('names', names, 'non-empty array');
    }

    const client = this.#getOrCreateClient('GetParameters');
    const request = { names };

    debug(
      `Getting ${names.length} parameter(s) from node ${this.#remoteNodeName}`
    );

    const response = await client.sendRequestAsync(request, {
      timeout: options.timeout || this.#timeout,
      signal: options.signal,
    });

    const parameters = [];
    for (let i = 0; i < names.length; i++) {
      const value = response.values[i];
      if (value.type !== ParameterType.PARAMETER_NOT_SET) {
        parameters.push(
          new Parameter(
            names[i],
            value.type,
            this.#deserializeParameterValue(value)
          )
        );
      }
    }

    debug(`Retrieved ${parameters.length} parameter(s)`);
    return parameters;
  }

  /**
   * Set a single parameter on the remote node.
   * @param {string} name - The name of the parameter to set.
   * @param {*} value - The value to set. Type is automatically inferred.
   * @param {object} [options] - Options for the service call.
   * @param {number} [options.timeout] - Timeout in milliseconds for this specific call.
   * @param {AbortSignal} [options.signal] - AbortSignal to cancel the request.
   * @return {Promise<object>} - Promise that resolves with the result {successful: boolean, reason: string}.
   * @throws {Error} If the service call fails.
   */
  async setParameter(name, value, options = {}) {
    this.#throwErrorIfClientDestroyed();

    const results = await this.setParameters([{ name, value }], options);
    return results[0];
  }

  /**
   * Set multiple parameters on the remote node.
   * @param {Array<{name: string, value: *}>} parameters - Array of parameter objects with name and value.
   * @param {object} [options] - Options for the service call.
   * @param {number} [options.timeout] - Timeout in milliseconds for this specific call.
   * @param {AbortSignal} [options.signal] - AbortSignal to cancel the request.
   * @return {Promise<Array<{name: string, successful: boolean, reason: string}>>} - Promise that resolves with an array of results.
   * @throws {Error} If the service call fails.
   */
  async setParameters(parameters, options = {}) {
    this.#throwErrorIfClientDestroyed();

    if (!Array.isArray(parameters) || parameters.length === 0) {
      throw new TypeValidationError(
        'parameters',
        parameters,
        'non-empty array'
      );
    }

    const client = this.#getOrCreateClient('SetParameters');
    const request = {
      parameters: parameters.map((param) => ({
        name: param.name,
        value: this.#serializeParameterValue(param.value),
      })),
    };

    debug(
      `Setting ${parameters.length} parameter(s) on node ${this.#remoteNodeName}`
    );

    const response = await client.sendRequestAsync(request, {
      timeout: options.timeout || this.#timeout,
      signal: options.signal,
    });

    const results = response.results.map((result, index) => ({
      name: parameters[index].name,
      successful: result.successful,
      reason: result.reason || '',
    }));

    debug(
      `Set ${results.filter((r) => r.successful).length}/${results.length} parameter(s) successfully`
    );
    return results;
  }

  /**
   * List all parameters available on the remote node.
   * @param {object} [options] - Options for listing parameters.
   * @param {string[]} [options.prefixes] - Optional array of parameter name prefixes to filter by.
   * @param {number} [options.depth=0] - Depth of parameter namespace to list (0 = unlimited).
   * @param {number} [options.timeout] - Timeout in milliseconds for this specific call.
   * @param {AbortSignal} [options.signal] - AbortSignal to cancel the request.
   * @return {Promise<{names: string[], prefixes: string[]}>} - Promise that resolves with parameter names and prefixes.
   * @throws {Error} If the service call fails.
   */
  async listParameters(options = {}) {
    this.#throwErrorIfClientDestroyed();

    const client = this.#getOrCreateClient('ListParameters');
    const request = {
      prefixes: options.prefixes || [],
      depth: options.depth !== undefined ? BigInt(options.depth) : BigInt(0),
    };

    debug(`Listing parameters on node ${this.#remoteNodeName}`);

    const response = await client.sendRequestAsync(request, {
      timeout: options.timeout || this.#timeout,
      signal: options.signal,
    });

    debug(
      `Listed ${response.result.names.length} parameter(s) and ${response.result.prefixes.length} prefix(es)`
    );

    return {
      names: response.result.names || [],
      prefixes: response.result.prefixes || [],
    };
  }

  /**
   * Describe parameters on the remote node.
   * @param {string[]} names - Array of parameter names to describe.
   * @param {object} [options] - Options for the service call.
   * @param {number} [options.timeout] - Timeout in milliseconds for this specific call.
   * @param {AbortSignal} [options.signal] - AbortSignal to cancel the request.
   * @return {Promise<Array<object>>} - Promise that resolves with an array of parameter descriptors.
   * @throws {Error} If the service call fails.
   */
  async describeParameters(names, options = {}) {
    this.#throwErrorIfClientDestroyed();

    if (!Array.isArray(names) || names.length === 0) {
      throw new TypeValidationError('names', names, 'non-empty array');
    }

    const client = this.#getOrCreateClient('DescribeParameters');
    const request = { names };

    debug(
      `Describing ${names.length} parameter(s) on node ${this.#remoteNodeName}`
    );

    const response = await client.sendRequestAsync(request, {
      timeout: options.timeout || this.#timeout,
      signal: options.signal,
    });

    debug(`Described ${response.descriptors.length} parameter(s)`);
    return response.descriptors || [];
  }

  /**
   * Get the types of parameters on the remote node.
   * @param {string[]} names - Array of parameter names.
   * @param {object} [options] - Options for the service call.
   * @param {number} [options.timeout] - Timeout in milliseconds for this specific call.
   * @param {AbortSignal} [options.signal] - AbortSignal to cancel the request.
   * @return {Promise<Array<number>>} - Promise that resolves with an array of parameter types.
   * @throws {Error} If the service call fails.
   */
  async getParameterTypes(names, options = {}) {
    this.#throwErrorIfClientDestroyed();

    if (!Array.isArray(names) || names.length === 0) {
      throw new TypeValidationError('names', names, 'non-empty array');
    }

    const client = this.#getOrCreateClient('GetParameterTypes');
    const request = { names };

    debug(
      `Getting types for ${names.length} parameter(s) on node ${this.#remoteNodeName}`
    );

    const response = await client.sendRequestAsync(request, {
      timeout: options.timeout || this.#timeout,
      signal: options.signal,
    });

    return response.types || [];
  }

  /**
   * Wait for the parameter services to be available on the remote node.
   * @param {number} [timeout] - Optional timeout in milliseconds.
   * @return {Promise<boolean>} - Promise that resolves to true if services are available.
   */
  async waitForService(timeout) {
    this.#throwErrorIfClientDestroyed();

    const client = this.#getOrCreateClient('GetParameters');
    return await client.waitForService(timeout);
  }

  /**
   * Check if the parameter client has been destroyed.
   * @return {boolean} - True if destroyed, false otherwise.
   */
  isDestroyed() {
    return this.#destroyed;
  }

  /**
   * Destroy the parameter client and clean up all service clients.
   * @return {undefined}
   */
  destroy() {
    if (this.#destroyed) {
      return;
    }

    debug(`Destroying ParameterClient for node ${this.#remoteNodeName}`);

    for (const [serviceType, client] of this.#clients.entries()) {
      try {
        this.#node.destroyClient(client);
        debug(`Destroyed client for service type: ${serviceType}`);
      } catch (error) {
        debug(
          `Error destroying client for service type ${serviceType}:`,
          error
        );
      }
    }

    this.#clients.clear();
    this.#destroyed = true;

    debug('ParameterClient destroyed');
  }

  /**
   * Get or create a service client for the specified service type.
   * @private
   * @param {string} serviceType - The service type (e.g., 'GetParameters', 'SetParameters').
   * @return {Client} - The service client.
   */
  #getOrCreateClient(serviceType) {
    if (this.#clients.has(serviceType)) {
      return this.#clients.get(serviceType);
    }

    const serviceName = `/${this.#remoteNodeName}/${this.#toSnakeCase(serviceType)}`;
    const serviceInterface = `rcl_interfaces/srv/${serviceType}`;

    debug(`Creating client for service: ${serviceName}`);

    const client = this.#node.createClient(serviceInterface, serviceName);
    this.#clients.set(serviceType, client);

    return client;
  }

  /**
   * Serialize a JavaScript value to a ParameterValue message.
   * @private
   * @param {*} value - The value to serialize.
   * @return {object} - The ParameterValue message.
   */
  #serializeParameterValue(value) {
    const type = parameterTypeFromValue(value);

    const paramValue = {
      type,
      bool_value: false,
      integer_value: BigInt(0),
      double_value: 0.0,
      string_value: '',
      byte_array_value: [],
      bool_array_value: [],
      integer_array_value: [],
      double_array_value: [],
      string_array_value: [],
    };

    switch (type) {
      case ParameterType.PARAMETER_BOOL:
        paramValue.bool_value = value;
        break;
      case ParameterType.PARAMETER_INTEGER:
        paramValue.integer_value =
          typeof value === 'bigint' ? value : BigInt(value);
        break;
      case ParameterType.PARAMETER_DOUBLE:
        paramValue.double_value = value;
        break;
      case ParameterType.PARAMETER_STRING:
        paramValue.string_value = value;
        break;
      case ParameterType.PARAMETER_BOOL_ARRAY:
        paramValue.bool_array_value = Array.from(value);
        break;
      case ParameterType.PARAMETER_INTEGER_ARRAY:
        paramValue.integer_array_value = Array.from(value).map((v) =>
          typeof v === 'bigint' ? v : BigInt(v)
        );
        break;
      case ParameterType.PARAMETER_DOUBLE_ARRAY:
        paramValue.double_array_value = Array.from(value);
        break;
      case ParameterType.PARAMETER_STRING_ARRAY:
        paramValue.string_array_value = Array.from(value);
        break;
      case ParameterType.PARAMETER_BYTE_ARRAY:
        paramValue.byte_array_value = Array.from(value).map((v) =>
          Math.trunc(v)
        );
        break;
    }

    return paramValue;
  }

  /**
   * Deserialize a ParameterValue message to a JavaScript value.
   * @private
   * @param {object} paramValue - The ParameterValue message.
   * @return {*} - The deserialized value.
   */
  #deserializeParameterValue(paramValue) {
    switch (paramValue.type) {
      case ParameterType.PARAMETER_BOOL:
        return paramValue.bool_value;
      case ParameterType.PARAMETER_INTEGER:
        return paramValue.integer_value;
      case ParameterType.PARAMETER_DOUBLE:
        return paramValue.double_value;
      case ParameterType.PARAMETER_STRING:
        return paramValue.string_value;
      case ParameterType.PARAMETER_BYTE_ARRAY:
        return Array.from(paramValue.byte_array_value || []);
      case ParameterType.PARAMETER_BOOL_ARRAY:
        return Array.from(paramValue.bool_array_value || []);
      case ParameterType.PARAMETER_INTEGER_ARRAY:
        return Array.from(paramValue.integer_array_value || []).map((v) =>
          typeof v === 'bigint' ? v : BigInt(v)
        );
      case ParameterType.PARAMETER_DOUBLE_ARRAY:
        return Array.from(paramValue.double_array_value || []);
      case ParameterType.PARAMETER_STRING_ARRAY:
        return Array.from(paramValue.string_array_value || []);
      case ParameterType.PARAMETER_NOT_SET:
      default:
        return null;
    }
  }

  /**
   * Convert a service type name from PascalCase to snake_case.
   * @private
   * @param {string} name - The name to convert.
   * @return {string} - The snake_case name.
   */
  #toSnakeCase(name) {
    return name.replace(/[A-Z]/g, (letter, index) => {
      return index === 0 ? letter.toLowerCase() : '_' + letter.toLowerCase();
    });
  }

  /**
   * Throws an error if the client has been destroyed.
   * @private
   * @throws {Error} If the client has been destroyed.
   */
  #throwErrorIfClientDestroyed() {
    if (this.#destroyed) {
      throw new OperationError('ParameterClient has been destroyed', {
        code: 'CLIENT_DESTROYED',
        entityType: 'parameter_client',
        entityName: this.#remoteNodeName,
      });
    }
  }
}

module.exports = ParameterClient;
