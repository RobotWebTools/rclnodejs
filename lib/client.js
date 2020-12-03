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

const rclnodejs = require('bindings')('rclnodejs');
const Entity = require('./entity.js');
const debug = require('debug')('rclnodejs:client');

/**
 * @class - Class representing a Client in ROS
 * @hideconstructor
 */

class Client extends Entity {
  constructor(handle, nodeHandle, serviceName, typeClass, options) {
    super(handle, typeClass, options);
    this._nodeHandle = nodeHandle;
    this._serviceName = serviceName;
    this._sequenceNumber = 0;
  }

  /**
   * This callback is called when a resopnse is sent back from service
   * @callback ResponseCallback
   * @param {Object} response - The response sent from the service
   * @see [Client.sendRequest]{@link Client#sendRequest}
   * @see [Node.createService]{@link Node#createService}
   * @see {@link Client}
   * @see {@link Service}
   */

  /**
   * Send the request and will be notified asynchronously if receiving the repsonse.
   * @param {object} request - The request to be submitted.
   * @param {ResponseCallback} callback - Thc callback function for receiving the server response.
   * @return {undefined}
   * @see {@link ResponseCallback}
   */
  sendRequest(request, callback) {
    let requestToSend =
      request instanceof this._typeClass.Request
        ? request
        : new this._typeClass.Request(request);
    if (typeof callback !== 'function') {
      throw new TypeError('Invalid argument');
    }

    let rawRequest = requestToSend.serialize();
    this._sequenceNumber = rclnodejs.sendRequest(this._handle, rawRequest);
    debug(`Client has sent a ${this._serviceName} request.`);
    this._callback = callback;
  }

  get sequenceNumber() {
    return this._sequenceNumber;
  }

  processResponse(response) {
    debug(`Client has received ${this._serviceName} response from service.`);
    this._callback(response.toPlainObject(this.typedArrayEnabled));
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
    return this._serviceName;
  }
}

module.exports = Client;
