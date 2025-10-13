// Copyright (c) 2025, The Robot Web Tools Contributors
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

const loader = require('./interface_loader.js');
const rclnodejs = require('./native_loader.js');
const Service = require('./service.js');

const {
  ParameterType,
  Parameter,
  ParameterDescriptor,
} = require('../lib/parameter.js');

// This class is used to create a TypeDescriptionService which can be used to
// retrieve information about types used by the node’s publishers, subscribers,
// services or actions.
class TypeDescriptionService {
  constructor(node) {
    this._node = node;
    this._serviceName = this._node.name() + '/get_type_description';
    this._typeDescriptionServiceHandle = rclnodejs.initTypeDescriptionService(
      this._node.handle
    );
    this._typeClass = loader.loadInterface(
      'type_description_interfaces/srv/GetTypeDescription'
    );
    this._typeDescriptionService = null;

    this._enabled = false;
    const startTypeDescriptionServiceParam = 'start_type_description_service';
    if (!node.hasParameter(startTypeDescriptionServiceParam)) {
      node.declareParameter(
        new Parameter(
          startTypeDescriptionServiceParam,
          ParameterType.PARAMETER_BOOL,
          true
        ),
        new ParameterDescriptor(
          startTypeDescriptionServiceParam,
          ParameterType.PARAMETER_BOOL,
          'If enabled, start the ~/get_type_description service.',
          true
        )
      );
    }
    const param = node.getParameter(startTypeDescriptionServiceParam);
    this._enabled = param.value;
  }

  start() {
    if (!this._enabled || this._typeDescriptionService) {
      return;
    }

    this._typeDescriptionService = new Service(
      this._node.handle,
      this._typeDescriptionServiceHandle,
      this._serviceName,
      this._typeClass,
      this._node._validateOptions(undefined),
      (request, response) => {
        const responseToBeSent = new this._typeClass.Response();
        const requestReceived = new this._typeClass.Request(request);
        rclnodejs.handleRequest(
          this._node.handle,
          requestReceived.serialize(),
          responseToBeSent.serialize()
        );
        responseToBeSent.deserialize(responseToBeSent.refObject);
        rclnodejs.sendResponse(
          this._typeDescriptionServiceHandle,
          responseToBeSent.serialize(),
          response._header
        );
        return null;
      }
    );
    this._node._services.push(this._typeDescriptionService);
    this._node.syncHandles();
  }

  /**
   * Get the node this
   * @return {Node} - The supported node.
   */
  get node() {
    return this._node;
  }

  static toTypeHash(topicTypeHash) {
    return `RIHS0${topicTypeHash.version}_${topicTypeHash.value.toString('hex')}`;
  }
}

module.exports = TypeDescriptionService;
