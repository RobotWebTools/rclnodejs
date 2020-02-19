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
const {
  Parameter, 
  PARAMETER_SEPARATOR } = require('./parameter.js');


class ParameterService {

  constructor(node) {
    this._node = node;
    this._isRunning = false;
  }

  get node() {
    return this._node;
  }
  
  isStarted() {
    return this._isRunning;
  }

  /**
   * Configure parameter services and start processing client requests.
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
      (request, response) => this.handleListParameters(request, response)
    );

    // create DescribeParameters service
    const describeParametersServiceName = nodeName + '/describe_parameters';
    this._node.createService(
      'rcl_interfaces/srv/DescribeParameters',
      describeParametersServiceName,
      (request, response) => {
        this.handleDescribeParameters(request, response);
      });

    // create GetParameters service
    const getParametersServiceName = nodeName + '/get_parameters';
    this._node.createService(
      'rcl_interfaces/srv/GetParameters',
      getParametersServiceName,
      (request, response) => {
        this.handleGetParameters(request, response);
      });

    // create SetParameters service
    const setParametersServiceName = nodeName + '/set_parameters';
    this._node.createService(
      'rcl_interfaces/srv/SetParameters',
      setParametersServiceName,
      (request, response) => {
        this.handleSetParameters(request, response);
      });

    // create SetParametersAtomically service
    const setParametersAtomicallyServiceName = nodeName + '/set_parameters_atomically';
    this._node.createService(
      'rcl_interfaces/srv/SetParametersAtomically',
      setParametersAtomicallyServiceName,
      (request, response) => {
        this.handleSetParametersAtomically(request, response);
      });
  }

  // ListParameters_Request = {
  //   DEPTH_RECURSIVE: 0,
  //   prefixes: string[],
  //   depth: number
  // };
  // type ListParametersResult = {
  //      names: string[],
  //      prefixes: string[]
  //    };
  handleListParameters(request, response) {
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
      prefixedNames = 
        prefixedNames.filter( 
          name => name.split(PARAMETER_SEPARATOR).length - 1 < request.depth
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
            if (! result.prefixes.includes(fullPrefix)) {
              result.prefixes.push(fullPrefix);
            }
            if (! request.prefixes.includes(prefix)) {
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
        if (! request.prefixes.includes(prefix)) {
          result.prefixes.push(prefix);
        }
      }
    }

    response.send(msg);
  };

  /**
   * Get a list of ParameterDescriptors. 
   * 
   * Request.names identifies the descriptors to get.
   * If request.names is empty, get all descriptors.
   * 
   * @param {DescribeParameters_Request} request - 
   * @param {DescribeParameters_Response} response - 
   * @return {undefined} - 
   */
  handleDescribeParameters(request, response) {
    const names = request.names;

    const msg = response.template; // ParameterDescriptor
    if (names.length > 0) {
      const descriptors = this._node.getParameterDescriptors(names);
      msg.descriptors = descriptors.map(descriptor => descriptor.toMessage());
    }
    response.send(msg);
  };

  /**
   * Get a list of ParameterValue. 
   * 
   * request.names identifies the parameter values to get.
   * If request.names is empty return the value of all parameters.
   * 
   * @param {GetParameters_Request} request - 
   * @param {GetParameters_Response} response - 
   * @return {undefined} - 
   */
  handleGetParameters(request, response) {
    const parameters = this._node.getParameters(request.names);
  
    const msg = response.template;
    msg.values = parameters.map((param) => param.toParameterValueMessage());
    response.send(msg);
  };

  /**
   * Set a list of parameters. 
   * 
   * @param {SetParameters_Request} request - 
   * @param {SetParameters_Response} response - 
   * @return {undefined} - 
   */
  handleSetParameters(request, response) {
    const parameters = request.parameters.map(paramMsg => 
      Parameter.fromParameterMessage(paramMsg));

    const msg = response.template;
    msg.results = this._node.setParameters(parameters);
    response.send(msg);
  };

  /**
   * Set a list of parameters atomically. 
   * 
   * @param {SetParameters_Request} request - 
   * @param {SetParameters_Response} response - 
   * @return {undefined} - 
   */
  handleSetParametersAtomically(request, response) {
    const parameters = request.parameters.map(paramMsg => 
      Parameter.fromParameterMessage(paramMsg));

    const msg = response.template;
    msg.result = this._node.setParametersAtomically(parameters); 
    response.send(msg);
  };

}

module.exports = ParameterService;

