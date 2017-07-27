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

class Service {
  constructor(handle, serviceName, typeClass, callback) {
    this._handle = handle;
    this._serviceName = serviceName;
    this._typeClass = typeClass;
    this._request = new typeClass.Request();
    this._callback = callback;
  }

  get handle() {
    return this._handle;
  }

  get takenRequest() {
    return this._request.toRawROS();
  }

  processRequest(headerHandle) {
    let response = this._callback(this._request, new this._typeClass.Response());
    let rawROSResponse = response.toRawROS();
    rclnodejs.sendResponse(this._handle, rawROSResponse, headerHandle);
  }

  static createService(nodeHandle, serviceName, typeClass, callback) {
    let type = typeClass.type();
    let handle = rclnodejs.createService(nodeHandle, serviceName, type.interfaceName, type.pkgName);
    return new Service(handle, serviceName, typeClass, callback);
  }
};

module.exports = Service;
