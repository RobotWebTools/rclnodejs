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
const {MessageTranslator} = require('./message_translator.js');

/**
 * @class - Class representing a Service in ROS
 * @hideconstructor
 */

class Service extends Entity {
  constructor(handle, serviceName, typeClass, callback, qos) {
    super(handle, typeClass, qos);
    this._serviceName = serviceName;
    this._request = new typeClass.Request();
    this._callback = callback;
    this.translator = new MessageTranslator(typeClass);
  }

  processRequest(headerHandle, request) {
    this._request.deserialize(request);
    debug('Service has received a request from client.');

    const requestObj = MessageTranslator.toPlainObject(this._request);
    const responseObj = MessageTranslator.toPlainObject(new this._typeClass.Response());
    const responseRet = this._callback(requestObj, responseObj);

    const rawROSResponse = this.translator.toROSResponse(responseRet).serialize();
    rclnodejs.sendResponse(this._handle, rawROSResponse, headerHandle);
    debug('Service has processed the request and sent the response.');
  }

  static createService(nodeHandle, serviceName, typeClass, callback, qos) {
    let type = typeClass.type();
    let handle = rclnodejs.createService(nodeHandle, serviceName, type.interfaceName, type.pkgName, qos);
    return new Service(handle, serviceName, typeClass, callback, qos);
  }
};

module.exports = Service;
