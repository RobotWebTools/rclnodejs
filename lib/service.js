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
const debug = require('debug')('rclnodejs:service');

/**
 * The response to client
 *
 * @property {Object} template - Get an empty response object that is being sent to client.
 * @property {Service} service - The service that this response object is attaching to.
 *
 * @hideconstructor
 */
class Response {
  constructor(service, header) {
    this._service = service;
    this._header = header;
    this.sent = false;
  }

  get template() {
    let response = new this._service._typeClass.Response();
    return response.toPlainObject();
  }

  get service() {
    return this._service;
  }

  /**
   * Send response to client (the service caller)
   * @param {object} response - The plain JavaScript representing the response.
        Note: you can use .template to get an empty result object.
   * @return {undefined}
   * @see {@link Response#template}
   */
  send(response) {
    let responseToReturn = new this._service._typeClass.Response(response);
    const rawResponse = responseToReturn.serialize();
    rclnodejs.sendResponse(this._service._handle, rawResponse, this._header);
    this.sent = true;
  }
}

/**
 * @class - Class representing a Service in ROS
 * @hideconstructor
 */

class Service extends Entity {
  constructor(handle, serviceName, typeClass, options, callback) {
    super(handle, typeClass, options);
    this._callback = callback;
  }

  processRequest(headerHandle, request) {
    debug(`Service has received a ${this._serviceName} request from client.`);

    const plainObj = request.toPlainObject(this.typedArrayEnabled);
    const response = new Response(this, headerHandle);
    let responseToReturn = this._callback(plainObj, response);

    if (!response.sent && responseToReturn) {
      responseToReturn = new this._typeClass.Response(responseToReturn);
      const rawResponse = responseToReturn.serialize();
      rclnodejs.sendResponse(this._handle, rawResponse, headerHandle);
    }

    debug(
      `Service has processed the ${this._serviceName} request and sent the response.`
    );
  }

  static createService(nodeHandle, serviceName, typeClass, options, callback) {
    let type = typeClass.type();
    let handle = rclnodejs.createService(
      nodeHandle,
      serviceName,
      type.interfaceName,
      type.pkgName,
      options.qos
    );
    return new Service(handle, serviceName, typeClass, options, callback);
  }

  /**
   * @type {string}
   */
  get serviceName() {
    const fullServiceName = rclnodejs.getServiceName(this._handle); // returns /my_node/myservice
    return fullServiceName.split('/').pop();
  }
}

module.exports = Service;
