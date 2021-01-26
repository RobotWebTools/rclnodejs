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

const { Parameter } = require('./parameter.js');
/**
 * NodeOptions specify configuration choices during the
 * node instantiation process.
 * @class
 */
class NodeOptions {
  /**
   * Create a new instance with default property values.
   * @constructor
   * @param {boolean} [startParameterServices=true]
   * @param {array} [parameterOverrides=[]]
   * @param {boolean} [automaticallyDeclareParametersFromOverrides=false]
   */
  constructor(
    startParameterServices = true,
    parameterOverrides = [],
    automaticallyDeclareParametersFromOverrides = false
  ) {
    this._startParameterServices = startParameterServices;
    this._parameterOverrides = parameterOverrides;
    this._automaticallyDeclareParametersFromOverrides = automaticallyDeclareParametersFromOverrides;
  }

  /**
   * Get the startParameterServices option.
   * Default value = true;
   * @returns {boolean} -
   */
  get startParameterServices() {
    return this._startParameterServices;
  }

  /**
   * Set startParameterServices.
   * @param {boolean} startFlag - True indicates for a node to start it's parameter_service.
   */
  set startParameterServices(startFlag) {
    this._startParameterServices = startFlag;
  }

  /**
   * Get the parameterOverrides.
   * @return {Parameter[]} - An array of Parameters that serve as overrides for a node's default
   *   parameters. Default = empty array [].
   */
  get parameterOverrides() {
    return this._parameterOverrides;
  }

  /**
   * Set the Parameters that will serve to override a node's default parameter settings.
   * Setting to null, reinitializes the parameterOverrides to an empty array.
   * @param {Parameter|Parameter[]} parameters - A single parameter or parameter[].
   *    Default is an empty array.
   */
  set parameterOverrides(parameters = []) {
    let povalue = parameters;

    if (!povalue) {
      povalue = [];
    } else if (povalue instanceof Parameter) {
      // a single parameter
      povalue = [povalue];
    } else if (!Array.isArray(parameters)) {
      throw TypeError('Expected Parameter[]');
    }

    this._parameterOverrides = povalue;
  }

  /**
   * Get the automaticallyDeclareParametersFromOverrides.
   *
   * @returns {boolean} - True indicates that a node shold declare declare parameters from
   *    it's parameter-overrides
   */
  get automaticallyDeclareParametersFromOverrides() {
    return this._automaticallyDeclareParametersFromOverrides;
  }

  /**
   * Set automaticallyDeclareParametersFromOverrides.
   *
   * @param {boolean} declareParamsFlag - When true, a node will declare parameters from all
   * parameter-overrides.
   */
  set automaticallyDeclareParametersFromOverrides(declareParamsFlag) {
    this._automaticallyDeclareParametersFromOverrides = declareParamsFlag;
  }

  /**
   * Return an instance configured with default options.
   * @returns {NodeOptions} - An instance with default values.
   */
  static get defaultOptions() {
    return new NodeOptions();
  }
}

module.exports = NodeOptions;
