/* eslint-disable max-depth */
// Copyright (c) 2020 Wayne Parrott. All rights reserved.
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
const { Parameter, PARAMETER_SEPARATOR } = require('./parameter.js');

/**
 * Implements the ros2 service interfaces for interacting with a node's parameters.
 *
 * The interfaces implemented are:
 *  rcl_interfaces/srv/ListParameters
 *  rcl_interfaces/srv/DescribeParameters
 *  rcl_interfaces/srv/GetParameters
 *  rcl_interfaces/srv/SetParameters
 *  rcl_interfaces/srv/SetParametersAtomically
 *
 * Call start() to begin receiving client request.
 * All service requests are forwarded to the node this service works for.
 * @class
 */
class ParameterService {
  /**
   * Create a new instance.
   * @constructor
   * @param {Node} node - The node these services will support.
   */
  constructor(node) {
    this._node = node;
    this._isRunning = false;
  }

  /**
   * Get the node this
   * @return {Node} - The supported node.
   */
  get node() {
    return this._node;
  }

  /**
   * Check if interface services are configured and accepting requests.
   * @return {boolean} - True if services are active; false otherwise.
   */
  isStarted() {
    return this._isRunning;
  }

  /**
   * Configure parameter services and begin processing client requests.
   * If this service is already started the request is ignored, i.e., a nop.
   *
   * @return {undefined}
   */
  start() {
    // do nothing if service is already running
    if (this._isRunning) return;

    this._isRunning = true;
    const nodeName = this.node.name();

    // create ListParameters service
    const listParametersServiceName = nodeName + '/list_parameters';
    this.node.createService(
      'rcl_interfaces/srv/ListParameters',
      listParametersServiceName,
      (request, response) => this._handleListParameters(request, response)
    );

    // create DescribeParameters service
    const describeParametersServiceName = nodeName + '/describe_parameters';
    this._node.createService(
      'rcl_interfaces/srv/DescribeParameters',
      describeParametersServiceName,
      (request, response) => {
        this._handleDescribeParameters(request, response);
      }
    );

    // create GetParameters service
    const getParametersServiceName = nodeName + '/get_parameters';
    this._node.createService(
      'rcl_interfaces/srv/GetParameters',
      getParametersServiceName,
      (request, response) => {
        this._handleGetParameters(request, response);
      }
    );

    // create SetParameters service
    const setParametersServiceName = nodeName + '/set_parameters';
    this._node.createService(
      'rcl_interfaces/srv/SetParameters',
      setParametersServiceName,
      (request, response) => {
        this._handleSetParameters(request, response);
      }
    );

    // create SetParametersAtomically service
    const setParametersAtomicallyServiceName =
      nodeName + '/set_parameters_atomically';
    this._node.createService(
      'rcl_interfaces/srv/SetParametersAtomically',
      setParametersAtomicallyServiceName,
      (request, response) => {
        this._handleSetParametersAtomically(request, response);
      }
    );
  }

  /**
   * Get a list of parameter names.
   *
   * The body of the response is a rcl_interfaces.msg.ListParametersResult.
   *
   * @param {ListParameters_Request} request - The client request.
   * @param {ListParameters_Response} response - The service response with
   *    a list of parameter names.
   * @return {undefined} -
   */
  _handleListParameters(request, response) {
    const DEPTH_RECURSIVE = 0;

    let prefixedNames = [];
    const parameterNames = this._node.getParameterNames();
    const msg = response.template;
    const result = msg.result;

    for (const paramName of parameterNames) {
      if (paramName.includes(PARAMETER_SEPARATOR)) {
        prefixedNames.push(paramName);
        continue;
      } else if (request.prefixes.length > 0) {
        for (const prefix of request.prefixes) {
          if (paramName.startsWith(prefix)) {
            result.names.push(paramName);
            break;
          }
        }
      } else {
        result.names.push(paramName);
      }
    }

    if (request.depth == 1) {
      response.send(msg);
      return;
    }

    if (request.depth != DEPTH_RECURSIVE) {
      prefixedNames = prefixedNames.filter(
        (name) => name.split(PARAMETER_SEPARATOR).length - 1 < request.depth
      );
    }

    for (const paramName of prefixedNames) {
      if (request.prefixes.length > 0) {
        for (const prefix of request.prefixes) {
          if (paramName.startsWith(prefix + PARAMETER_SEPARATOR)) {
            result.names.push(paramName);

            // drop the last segment of paramName to reveal a prefix
            let fullPrefix = paramName.split(PARAMETER_SEPARATOR);
            fullPrefix.pop();
            fullPrefix = fullPrefix.join(PARAMETER_SEPARATOR);
            if (!result.prefixes.includes(fullPrefix)) {
              result.prefixes.push(fullPrefix);
            }
            if (!request.prefixes.includes(prefix)) {
              result.prefixes.push(prefix);
            }
          }
        }
      } else {
        result.names.push(paramName);

        // drop the last segment of paramName to reveal a prefix
        let prefix = paramName.split(PARAMETER_SEPARATOR);
        prefix.pop();
        prefix = prefix.join(PARAMETER_SEPARATOR);
        if (!request.prefixes.includes(prefix)) {
          result.prefixes.push(prefix);
        }
      }
    }

    response.send(msg);
  }

  /**
   * Get a list of ParameterDescriptors.
   *
   * Request.names identifies the descriptors to get.
   * If request.names is empty, get all descriptors.
   * The body of the response is a rcl_interfaces.msg.DescribeParametersResult.
   *
   * @param {DescribeParameters_Request} request - The client request
   * @param {DescribeParameters_Response} response - The server response with
   *    an array of ParameterDescriptors.
   * @return {undefined} -
   */
  _handleDescribeParameters(request, response) {
    const names = request.names;

    const msg = response.template; // ParameterDescriptor
    if (names.length > 0) {
      const descriptors = this._node.getParameterDescriptors(names);
      msg.descriptors = descriptors.map((descriptor) => descriptor.toMessage());
    }
    response.send(msg);
  }

  /**
   * Get a list of ParameterValue.
   *
   * request.names identifies the parameter values to get.
   * If request.names is empty return the value of all parameters.
   * The body of the response is a rcl_interfaces.msg.ParameterValue[].
   *
   * @param {GetParameters_Request} request - The client request.
   * @param {GetParameters_Response} response - The service response with
   *    an array of ParameterValue.
   * @return {undefined} -
   */
  _handleGetParameters(request, response) {
    const parameters = this._node.getParameters(request.names);

    const msg = response.template;
    msg.values = parameters.map((param) => param.toParameterValueMessage());
    response.send(msg);
  }

  /**
   * Update a list of parameters.
   *
   * Process each setParameter operation in the order defined by the request.
   * The result is an rcl_interfaces.msg.SetParametersResult[], one result
   * for each parameter.
   *
   * @param {SetParameters_Request} request - The client request.
   * @param {SetParameters_Response} response - The service response
   *    with a SetParametersResult[]
   * @return {undefined} -
   */
  _handleSetParameters(request, response) {
    const parameters = request.parameters.map((paramMsg) =>
      Parameter.fromParameterMessage(paramMsg)
    );

    const msg = response.template;
    msg.results = this._node.setParameters(parameters);
    response.send(msg);
  }

  /**
   * Update a list of parameters atomically.
   *
   * The body of the response is a rcl_interfaces.msg.SetParametersResult
   *
   * @param {SetParameters_Request} request - The client request.
   * @param {SetParameters_Response} response - The service response
   *    with a single SetParametersResult for the entire process.
   * @return {undefined} -
   */
  _handleSetParametersAtomically(request, response) {
    const parameters = request.parameters.map((paramMsg) =>
      Parameter.fromParameterMessage(paramMsg)
    );

    const msg = response.template;
    msg.result = this._node.setParametersAtomically(parameters);
    response.send(msg);
  }
}

module.exports = ParameterService;
