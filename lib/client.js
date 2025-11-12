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

const rclnodejs = require('./native_loader.js');
const DistroUtils = require('./distro.js');
const Entity = require('./entity.js');
const {
  TypeValidationError,
  TimeoutError,
  AbortError,
} = require('./errors.js');
const debug = require('debug')('rclnodejs:client');

// Polyfill for AbortSignal.any() for Node.js <= 20.3.0
// AbortSignal.any() was added in Node.js 20.3.0
// See https://developer.mozilla.org/en-US/docs/Web/API/AbortSignal/any_static
if (!AbortSignal.any) {
  AbortSignal.any = function (signals) {
    // Filter out null/undefined values and validate inputs
    const validSignals = Array.isArray(signals)
      ? signals.filter((signal) => signal != null)
      : [];

    // If no valid signals, return a never-aborting signal
    if (validSignals.length === 0) {
      return new AbortController().signal;
    }

    const controller = new AbortController();
    const listeners = [];

    // Cleanup function to remove all event listeners
    const cleanup = () => {
      listeners.forEach(({ signal, listener }) => {
        signal.removeEventListener('abort', listener);
      });
    };

    for (const signal of validSignals) {
      if (signal.aborted) {
        cleanup();
        controller.abort(signal.reason);
        return controller.signal;
      }

      const listener = () => {
        cleanup();
        controller.abort(signal.reason);
      };

      signal.addEventListener('abort', listener);
      listeners.push({ signal, listener });
    }

    return controller.signal;
  };
}

/**
 * @class - Class representing a Client in ROS
 * @hideconstructor
 */

class Client extends Entity {
  constructor(handle, nodeHandle, serviceName, typeClass, options) {
    super(handle, typeClass, options);
    this._nodeHandle = nodeHandle;
    this._serviceName = serviceName;
    this._sequenceNumberToCallbackMap = new Map();
  }

  /**
   * This callback is called when a response is sent back from service
   * @callback ResponseCallback
   * @param {Object} response - The response sent from the service
   * @see [Client.sendRequest]{@link Client#sendRequest}
   * @see [Node.createService]{@link Node#createService}
   * @see {@link Client}
   * @see {@link Service}
   */

  /**
   * Send the request and will be notified asynchronously if receiving the response.
   * @param {object} request - The request to be submitted.
   * @param {ResponseCallback} callback - Thc callback function for receiving the server response.
   * @return {undefined}
   * @see {@link ResponseCallback}
   */
  sendRequest(request, callback) {
    if (typeof callback !== 'function') {
      throw new TypeValidationError('callback', callback, 'function', {
        entityType: 'service',
        entityName: this._serviceName,
      });
    }

    let requestToSend =
      request instanceof this._typeClass.Request
        ? request
        : new this._typeClass.Request(request);

    let rawRequest = requestToSend.serialize();
    let sequenceNumber = rclnodejs.sendRequest(this._handle, rawRequest);
    debug(`Client has sent a ${this._serviceName} request.`);
    this._sequenceNumberToCallbackMap.set(sequenceNumber, callback);
  }

  /**
   * Send the request and return a Promise that resolves with the response.
   * @param {object} request - The request to be submitted.
   * @param {object} [options] - Optional parameters for the request.
   * @param {number} [options.timeout] - Timeout in milliseconds for the request.
   * @param {AbortSignal} [options.signal] - AbortSignal to cancel the request.
   * @return {Promise<object>} Promise that resolves with the service response.
   * @throws {module:rclnodejs.TimeoutError} If the request times out (when options.timeout is exceeded).
   * @throws {module:rclnodejs.AbortError} If the request is manually aborted (via options.signal).
   * @throws {Error} If the request fails for other reasons.
   */
  sendRequestAsync(request, options = {}) {
    return new Promise((resolve, reject) => {
      let sequenceNumber = null;
      let isResolved = false;
      let isTimeout = false;

      const cleanup = () => {
        if (sequenceNumber !== null) {
          this._sequenceNumberToCallbackMap.delete(sequenceNumber);
        }
        isResolved = true;
      };

      let effectiveSignal = options.signal;

      if (options.timeout !== undefined && options.timeout >= 0) {
        const timeoutSignal = AbortSignal.timeout(options.timeout);

        timeoutSignal.addEventListener('abort', () => {
          isTimeout = true;
        });

        if (options.signal) {
          effectiveSignal = AbortSignal.any([options.signal, timeoutSignal]);
        } else {
          effectiveSignal = timeoutSignal;
        }
      }

      if (effectiveSignal) {
        if (effectiveSignal.aborted) {
          const error = isTimeout
            ? new TimeoutError('Service request', options.timeout, {
                entityType: 'service',
                entityName: this._serviceName,
              })
            : new AbortError('Service request', undefined, {
                entityType: 'service',
                entityName: this._serviceName,
              });
          reject(error);
          return;
        }

        effectiveSignal.addEventListener('abort', () => {
          if (!isResolved) {
            cleanup();
            const error = isTimeout
              ? new TimeoutError('Service request', options.timeout, {
                  entityType: 'service',
                  entityName: this._serviceName,
                })
              : new AbortError('Service request', undefined, {
                  entityType: 'service',
                  entityName: this._serviceName,
                });
            reject(error);
          }
        });
      }

      try {
        let requestToSend =
          request instanceof this._typeClass.Request
            ? request
            : new this._typeClass.Request(request);

        let rawRequest = requestToSend.serialize();
        sequenceNumber = rclnodejs.sendRequest(this._handle, rawRequest);

        debug(`Client has sent a ${this._serviceName} request (async).`);

        this._sequenceNumberToCallbackMap.set(sequenceNumber, (response) => {
          if (!isResolved) {
            cleanup();
            resolve(response);
          }
        });
      } catch (error) {
        cleanup();
        reject(error);
      }
    });
  }

  processResponse(sequenceNumber, response) {
    if (this._sequenceNumberToCallbackMap.has(sequenceNumber)) {
      debug(`Client has received ${this._serviceName} response from service.`);
      let callback = this._sequenceNumberToCallbackMap.get(sequenceNumber);
      this._sequenceNumberToCallbackMap.delete(sequenceNumber);
      callback(response.toPlainObject(this.typedArrayEnabled));
    } else {
      debug(
        `Client has received an unexpected ${this._serviceName} with sequence number ${sequenceNumber}.`
      );
    }
  }

  /**
   * Checks if the service is available.
   * @return {boolean} true if the service is available.
   */
  isServiceServerAvailable() {
    return rclnodejs.serviceServerIsAvailable(this._nodeHandle, this.handle);
  }

  /**
   * Wait until the service server is available or a timeout is reached. This
   * function polls for the service state so it may not return as soon as the
   * service is available.
   * @param {number} timeout The maximum amount of time to wait for, if timeout
   * is `undefined` or `< 0`, this will wait indefinitely.
   * @return {Promise<boolean>} true if the service is available.
   */
  async waitForService(timeout = undefined) {
    let deadline = Infinity;
    if (timeout !== undefined && timeout >= 0) {
      deadline = Date.now() + timeout;
    }
    let waitMs = 5;
    let serviceAvailable = this.isServiceServerAvailable();
    while (!serviceAvailable && Date.now() < deadline) {
      waitMs *= 2;
      waitMs = Math.min(waitMs, 1000);
      if (timeout !== undefined && timeout >= -1) {
        waitMs = Math.min(waitMs, deadline - Date.now());
      }
      await new Promise((resolve) => setTimeout(resolve, waitMs));
      serviceAvailable = this.isServiceServerAvailable();
    }
    return serviceAvailable;
  }

  static createClient(nodeHandle, serviceName, typeClass, options) {
    let type = typeClass.type();
    let handle = rclnodejs.createClient(
      nodeHandle,
      serviceName,
      type.interfaceName,
      type.pkgName,
      options.qos
    );
    return new Client(handle, nodeHandle, serviceName, typeClass, options);
  }

  /**
   * @type {string}
   */
  get serviceName() {
    return rclnodejs.getClientServiceName(this._handle);
  }

  /**
   * Configure introspection.
   * @param {Clock} clock - Clock to use for service event timestamps
   * @param {QoS} qos - QoSProfile for the service event publisher
   * @param {ServiceIntrospectionState} introspectionState - State to set introspection to
   */
  configureIntrospection(clock, qos, introspectionState) {
    if (DistroUtils.getDistroId() <= DistroUtils.getDistroId('humble')) {
      console.warn(
        'Service introspection is not supported by this version of ROS 2'
      );
      return;
    }

    let type = this.typeClass.type();
    rclnodejs.configureClientIntrospection(
      this.handle,
      this._nodeHandle,
      clock.handle,
      type.interfaceName,
      type.pkgName,
      qos,
      introspectionState
    );
  }

  /**
   * Get the logger name for this client.
   * @returns {string} The logger name for this client.
   */
  get loggerName() {
    return rclnodejs.getNodeLoggerName(this._nodeHandle);
  }
}

module.exports = Client;
