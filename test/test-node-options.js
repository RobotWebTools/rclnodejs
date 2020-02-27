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
const IsClose = require('is-close');

const rclnodejs = require('../index.js');
const assertUtils = require('./utils.js');
const assertThrowsError = assertUtils.assertThrowsError;

const NodeOptions = rclnodejs.NodeOptions;

const {
  ParameterType,
  Parameter,
  ParameterDescriptor,
} = require('../lib/parameter.js');

describe('rclnodejs NodeOptions test suite', function() {
  it('constructor test', function() {
    const nodeOptions = new NodeOptions();

    assert.strictEqual(nodeOptions.startParameterServices, true);
    assert.strictEqual(
      nodeOptions.automaticallyDeclareParametersFromOverrides,
      false
    );
    assert.ok(Array.isArray(nodeOptions.parameterOverrides));
    assert.strictEqual(nodeOptions.parameterOverrides.length, 0);
  });

  it('defaultOptions test', function() {
    const nodeOptions = NodeOptions.defaultOptions;

    assert.strictEqual(nodeOptions.startParameterServices, true);
    assert.strictEqual(
      nodeOptions.automaticallyDeclareParametersFromOverrides,
      false
    );
    assert.ok(Array.isArray(nodeOptions.parameterOverrides));
    assert.strictEqual(nodeOptions.parameterOverrides.length, 0);
  });

  it('setters/getters test', function() {
    const nodeOptions = new NodeOptions();
    const param = new Parameter(
      'str_param',
      ParameterType.PARAMETER_STRING,
      'foobar'
    );

    nodeOptions.startParameterServices = false;
    nodeOptions.automaticallyDeclareParametersFromOverrides = true;
    nodeOptions.parameterOverrides = param;

    assert.strictEqual(nodeOptions.startParameterServices, false);
    assert.strictEqual(
      nodeOptions.automaticallyDeclareParametersFromOverrides,
      true
    );
    assert.ok(Array.isArray(nodeOptions.parameterOverrides));
    assert.strictEqual(nodeOptions.parameterOverrides.length, 1);
    assert.strictEqual(nodeOptions.parameterOverrides[0].name, 'str_param');

    nodeOptions.parameterOverrides = null;
    assert.ok(Array.isArray(nodeOptions.parameterOverrides));
    assert.strictEqual(nodeOptions.parameterOverrides.length, 0);

    nodeOptions.parameterOverrides = [param];
    assert.strictEqual(nodeOptions.parameterOverrides.length, 1);
    assert.strictEqual(nodeOptions.parameterOverrides[0].name, 'str_param');

    assertThrowsError(
      () => (nodeOptions.parameterOverrides = new Object()),
      TypeError,
      'Expected Parameter*'
    );
  });
});
