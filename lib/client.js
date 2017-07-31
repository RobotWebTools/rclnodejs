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

/** Class representing a Client in ROS */
class Client {
  constructor(handle, serviceName, typeClass) {
    this._handle = handle;
    this._typeClass = typeClass;
    this._response = new typeClass.Response();
    this._serviceName = serviceName;
    this._sequenceNumber = 0;
  }

  /**
   * Send the request and will be notified asynchronously if receiving the repsonse.
   * @param {object} request - The request to be submitted.
   * @param {function} callback - Thc callback function.
   * @return {undefined}
   */
  sendRequest(request, callback) {
    if (typeof (request) !== 'object' || typeof (callback) !== 'function') {
      throw new TypeError('Invalid argument');
    }

    let rawROSRequest = request.toRawROS();
    this._sequenceNumber = rclnodejs.sendRequest(this._handle, rawROSRequest);
    this._callback = callback;
  }

  get handle() {
    return this._handle;
  }

  get takenResponse() {
    return this._response.toRawROS();
  }

  get sequenceNumber() {
    return this._sequenceNumber;
  }

  processResponse() {
    this._callback(this._response);
  }

  static createClient(nodeHandle, serviceName, typeClass) {
    let type = typeClass.type();
    let handle = rclnodejs.createClient(nodeHandle, serviceName, type.interfaceName, type.pkgName);
    return new Client(handle, serviceName, typeClass);
  }
};

module.exports = Client;
