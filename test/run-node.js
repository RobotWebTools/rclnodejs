// Copyright (c) Wayne Parrott All rights reserved.
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

const assert = require('assert');

const rclnodejs = require('../index.js');
const assertUtils = require('./utils.js');
const assertThrowsError = assertUtils.assertThrowsError;

const Parameters = require('../lib/parameters.js');
const Parameter = Parameters.Parameter;
const ParameterType = Parameters.ParameterType;
const ParameterDescriptor = Parameters.ParameterDescriptor;
const IntegerRange = Parameters.IntegerRange;
const FloatingPointRange = Parameters.FloatingPointRange;
const PARAM_REL_TOL = 1e-6;

const Node = rclnodejs.Node;
const NodeOptions = Node.NodeOptions;
const Context = rclnodejs.Context;


describe('RLCNODE RUN (5 mins)', function () {
  const NODE_NAME = 'test_node';

  let node;
  this.timeout(5 * 60 * 1000);

  afterEach(function () {
    if (node) node.destroy();
    rclnodejs.shutdown();
  });

  it('RUUUUN', async function () {

    await rclnodejs.init();

    node = rclnodejs.createNode(NODE_NAME);
    rclnodejs.spin(node);

    node.declareParameter(
      new Parameter('p1', ParameterType.PARAMETER_STRING, 'helloworld'),
      new ParameterDescriptor('p1', ParameterType.PARAMETER_STRING)
    );

    const parameters = node.getParameters();
    
    assert.ok(Array.isArray(parameters), 'expected only 1 parameter');
    assert.equal(parameters[0].name, 'p1', 'expected parameter.name == p1');
    assert.equal(parameters[0].value, 'helloworld', 'expected parameterOverride.value == helloworld');

    await new Promise(resolve => setTimeout(resolve, 50000));
  });

});