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
const assertUtils = require('./utils.js');
const assertThrowsError = assertUtils.assertThrowsError;
const IsClose = require('is-close');
const rclnodejs = require('../index.js');

const ParameterType = rclnodejs.ParameterType;
const Parameter = rclnodejs.Parameter;
const ParameterDescriptor = rclnodejs.ParameterDescriptor;
const FloatingPointRange = rclnodejs.FloatingPointRange;
const IntegerRange = rclnodejs.IntegerRange;
const DEFAULT_NUMERIC_RANGE_TOLERANCE =
  rclnodejs.DEFAULT_NUMERIC_RANGE_TOLERANCE;
const NodeOptions = rclnodejs.NodeOptions;
const Context = rclnodejs.Context;

const STRING_DESCRIPTOR = new ParameterDescriptor(
  'strD',
  ParameterType.PARAMETER_STRING
);
const STRING_READONLY_DESCRIPTOR = new ParameterDescriptor(
  'strDRO',
  ParameterType.PARAMETER_STRING,
  'basic description',
  true
);
const BOOL_DESCRIPTOR = new ParameterDescriptor(
  'boolD',
  ParameterType.PARAMETER_BOOL
);
const INTEGER_DESCRIPTOR = new ParameterDescriptor(
  'intD',
  ParameterType.PARAMETER_INTEGER
);
const DOUBLE_DESCRIPTOR = new ParameterDescriptor(
  'dblD',
  ParameterType.PARAMETER_DOUBLE
);
const STRING_ARRAY_DESCRIPTOR = new ParameterDescriptor(
  'strArrD',
  ParameterType.PARAMETER_STRING_ARRAY
);
const BOOL_ARRAY_DESCRIPTOR = new ParameterDescriptor(
  'boolArrD',
  ParameterType.PARAMETER_BOOL_ARRAY
);
const BYTE_ARRAY_DESCRIPTOR = new ParameterDescriptor(
  'byteArrD',
  ParameterType.PARAMETER_BYTE_ARRAY
);
const INTEGER_ARRAY_DESCRIPTOR = new ParameterDescriptor(
  'intArrD',
  ParameterType.PARAMETER_INTEGER_ARRAY
);
const DOUBLE_ARRAY_DESCRIPTOR = new ParameterDescriptor(
  'dbleArrD',
  ParameterType.PARAMETER_DOUBLE_ARRAY
);

describe('rclnodejs parameters test suite', function() {
  describe('parameter api tests', function() {
    it('Parameter constructor', function() {
      let param = new Parameter(
        'str_param',
        ParameterType.PARAMETER_STRING,
        'foobar'
      );

      assert.equal(param.name, 'str_param');
      assert.equal(param.type, ParameterType.PARAMETER_STRING);
      assert.equal(param.value, 'foobar');
    });

    it('Parameter update', function() {
      let param = new Parameter(
        'int_param',
        ParameterType.PARAMETER_INTEGER,
        100
      );
      assert.strictEqual(param.value, 100);

      param.value = 101;
      assert.strictEqual(param.value, 101);

      assertThrowsError(() => (param.value = 'hello world'), TypeError);
    });
  });

  describe('range tests', function() {
    it('Math IsClose test', function() {
      assert.ok(IsClose.isClose(1.0, 1.0, DEFAULT_NUMERIC_RANGE_TOLERANCE));
      assert.ok(!IsClose.isClose(1.0, 1.1, DEFAULT_NUMERIC_RANGE_TOLERANCE));
      assert.ok(IsClose.isClose(1.0, 1.1, 0.11));
    });

    it('IntegerRange test', function() {
      const range = new IntegerRange(0, 100, 1);

      // test property accessors
      assert.equal(range.fromValue, 0);
      assert.equal(range.toValue, 100);
      assert.equal(range.step, 1);

      // test midpoint
      assert.ok(range.inRange(50));

      // test lower boundary
      assert.ok(range.inRange(1));
      assert.ok(range.inRange(0));
      assert.ok(!range.inRange(-1));

      // test upper boundary
      assert.ok(range.inRange(99));
      assert.ok(range.inRange(100));
      assert.ok(!range.inRange(101));
    });

    it('FloatingPointRange test', function() {
      const range = new FloatingPointRange(-10.0, 10.0, 0.25);

      // test property accessors
      assert.equal(range.fromValue, -10);
      assert.equal(range.toValue, 10);
      assert.equal(range.step, 0.25);

      // test midpoint
      assert.ok(range.inRange(0.0));

      // test lower boundary
      assert.ok(range.inRange(-9));
      assert.ok(range.inRange(-10));
      assert.ok(!range.inRange(-11));

      // test upper boundary
      assert.ok(range.inRange(9));
      assert.ok(range.inRange(10));
      assert.ok(range.inRange(10.000009));
      assert.ok(range.inRange(10.00001));
      assert.ok(!range.inRange(10.00002));
    });
  });

  describe('parameter-descriptor tests', function() {
    it('String descriptor test', function() {
      assert.equal('strD', STRING_DESCRIPTOR.name);
      assert.equal(ParameterType.PARAMETER_STRING, STRING_DESCRIPTOR.type);
      assert.equal('no description', STRING_DESCRIPTOR.description);
      assert.ok(!STRING_DESCRIPTOR.readOnly);

      assert.equal('strDRO', STRING_READONLY_DESCRIPTOR.name);
      assert.equal('basic description', STRING_READONLY_DESCRIPTOR.description);
      assert.ok(STRING_READONLY_DESCRIPTOR.readOnly);
    });

    it('Integer descriptor with [0-255] range test', function() {
      const descriptor = new ParameterDescriptor(
        'int_param',
        ParameterType.PARAMETER_INTEGER
      );
      descriptor.range = new IntegerRange(0, 255);

      const param = new Parameter(
        'int_param',
        ParameterType.PARAMETER_INTEGER,
        100
      );
      assert.ifError(descriptor.validateParameter(param));

      param.value = 255;
      assert.ifError(descriptor.validateParameter(param));

      param.value = -1;
      assertThrowsError(() => descriptor.validateParameter(param), RangeError);

      param.value = 256;
      assertThrowsError(() => descriptor.validateParameter(param), RangeError);
    });

    it('Integer descriptor with [0-255], step=5 range test', function() {
      const descriptor = new ParameterDescriptor(
        'int_param',
        ParameterType.PARAMETER_INTEGER
      );
      descriptor.range = new IntegerRange(0, 255, 5);

      const param = new Parameter(
        'int_param',
        ParameterType.PARAMETER_INTEGER,
        100
      );
      assert.ifError(descriptor.validateParameter(param));

      param.value = 255;
      assert.ifError(descriptor.validateParameter(param));

      param.value = 1;
      assertThrowsError(() => descriptor.validateParameter(param), RangeError);

      param.value = 256;
      assertThrowsError(() => descriptor.validateParameter(param), RangeError);
    });
  });

  describe('rcl parameter-overrides test', function() {
    const NODE_NAME = 'test_node';

    let node;
    this.timeout(60 * 1000);

    afterEach(function() {
      if (node) node.destroy();
      rclnodejs.shutdown();
    });

    it('cli parameter-override: 1 global param', async function() {
      await rclnodejs.init(Context.defaultContext(), [
        '--ros-args',
        '-p',
        'p1:=foobar',
      ]);

      node = rclnodejs.createNode(NODE_NAME);
      const overrides = node.getParameterOverrides();
      assert.equal(overrides.length, 1, 'expected only 1 parameter');
      assert.equal(
        overrides[0].name,
        'p1',
        'expected parameterOverride.name == p1'
      );
      assert.equal(
        overrides[0].value,
        'foobar',
        'expected parameterOverride.value == foobar'
      );
    });

    it('cli parameter-override: 1 global param redefined', async function() {
      await rclnodejs.init(Context.defaultContext(), [
        '--ros-args',
        '-p',
        'p1:=foobar',
        '-p',
        'p1:=helloworld',
      ]);

      node = rclnodejs.createNode(NODE_NAME);
      const overrides = node.getParameterOverrides();
      assert.equal(overrides.length, 1, 'expected only 1 parameter');
      assert.equal(
        overrides[0].name,
        'p1',
        'expected parameterOverride.name == p1'
      );
      assert.equal(
        overrides[0].value,
        'helloworld',
        'expected parameterOverride.value == helloworld'
      );
    });

    it('cli parameter override, global & node:param', async function() {
      await rclnodejs.init(Context.defaultContext(), [
        '--ros-args',
        '-p',
        'p1:=foobar',
        '-p',
        NODE_NAME + ':p2:=123',
      ]);

      node = rclnodejs.createNode(NODE_NAME);
      const overrides = node.getParameterOverrides();
      assert.equal(overrides.length, 2, 'expected only 1 parameter');
      assert.equal(
        overrides[0].name,
        'p1',
        'expected parameterOverride.name == p1'
      );
      assert.equal(
        overrides[0].value,
        'foobar',
        'expected parameterOverride.value == foobar'
      );

      assert.equal(
        overrides[1].name,
        'p2',
        'expected parameterOverride.name == p2'
      );
      assert.equal(
        overrides[1].value,
        '123',
        'expected parameterOverride.value == 123'
      );
    });

    it('cli parameter override, filter out other node parameters', async function() {
      await rclnodejs.init(Context.defaultContext(), [
        '--ros-args',
        '-p',
        NODE_NAME + ':p1:=foobar',
        '-p',
        'xxx:p1:=foobar',
      ]);

      node = rclnodejs.createNode(NODE_NAME);
      const overrides = node.getParameterOverrides();
      assert.equal(overrides.length, 1, 'expected only 1 parameter');
      assert.equal(
        overrides[0].name,
        'p1',
        'expected parameterOverride.name == p1'
      );
      assert.equal(
        overrides[0].value,
        'foobar',
        'expected parameterOverride.value == foobar'
      );
    });

    it('cli + constructor parameter overrides', async function() {
      await rclnodejs.init(Context.defaultContext(), [
        '--ros-args',
        '-p',
        'p1:=foobar',
      ]);

      const options = new NodeOptions();
      options.parameterOverrides = [
        new Parameter('p1', ParameterType.PARAMETER_STRING, 'helloworld'),
      ];
      node = rclnodejs.createNode(
        NODE_NAME,
        '',
        Context.defaultContext(),
        options
      );
      const overrides = node.getParameterOverrides();
      assert.equal(overrides.length, 1, 'expected only 1 parameter');
      assert.equal(
        overrides[0].name,
        'p1',
        'expected parameterOverride.name == p1'
      );
      assert.equal(
        overrides[0].value,
        'helloworld',
        'expected parameterOverride.value == helloworld'
      );
    });

    it('cli load parameter.yaml file ', async function() {
      await rclnodejs.init(Context.defaultContext(), [
        '--ros-args',
        '--params-file',
        'test/yaml/test_parameters.1.yaml',
      ]);

      node = rclnodejs.createNode(NODE_NAME);

      const overrides = node.getParameterOverrides();
      assert.equal(overrides.length, 2, 'expected only 2 parameter');
      assert.equal(
        overrides[0].name,
        'int_param',
        'expected parameterOverride.name == int_param'
      );
      assert.equal(
        overrides[0].value,
        1,
        'expected parameterOverride.value == 1'
      );
      assert.equal(
        overrides[1].name,
        'param_group.string_param',
        'expected parameterOverride.name == param_group.string_param'
      );
      assert.equal(
        overrides[1].value,
        'foo',
        'expected parameterOverride.value == foo'
      );
    });

    it('cli load from parameter.yaml file with global /** param group ', async function() {
      await rclnodejs.init(Context.defaultContext(), [
        '--ros-args',
        '--params-file',
        'test/yaml/test_parameters.2.yaml',
      ]);

      node = rclnodejs.createNode(NODE_NAME);

      const overrides = node.getParameterOverrides();

      assert.equal(overrides.length, 2, 'expected only 2 parameter');
      assert.equal(
        overrides[0].name,
        'int_param',
        'expected parameterOverride.name == int_param'
      );
      assert.equal(
        overrides[0].value,
        3,
        'expected parameterOverride.value == 1'
      );
      assert.equal(
        overrides[1].name,
        'bool_param',
        'expected parameterOverride.name == bool_param'
      );
      assert.equal(
        overrides[1].value,
        true,
        'expected bool_param.value == true'
      );
    });

    it('override param from parameter.yaml file ', async function() {
      await rclnodejs.init(Context.defaultContext(), [
        '--ros-args',
        '--params-file',
        'test/yaml/test_parameters.1.yaml',
        '-p',
        'test_node:int_param:=2',
      ]);

      node = rclnodejs.createNode(NODE_NAME);

      const overrides = node.getParameterOverrides();
      assert.equal(overrides.length, 2, 'expected only 2 parameter');
      assert.equal(
        overrides[0].name,
        'int_param',
        'expected parameterOverride.name == int_param'
      );
      assert.equal(
        overrides[0].value,
        2,
        'expected parameterOverride.value == 2'
      );
    });
  });

  describe('parameter onSetParameterCallback tests', function() {
    const NODE_NAME = 'test_node';
    const PARAM_NAME = 'TEST_PARAM';
    const PARAM_TYPE = ParameterType.PARAMETER_STRING;
    const PARAM_VALUE = 'helloworld';
    let PARAMETER = undefined;
    const SET_PARAMETER_RESULT_SUCCESS = { successful: true, reason: null };
    const FAIL_REASON = 'FAIL';
    const SET_PARAMETER_RESULT_FAIL = {
      successful: false,
      reason: FAIL_REASON,
    };

    let node;
    this.timeout(60 * 1000);

    /* eslint-disable-next-line space-before-function-paren */
    before(async () => {
      await rclnodejs.init();
    });

    /* eslint-disable-next-line space-before-function-paren */
    this.beforeEach(async () => {
      node = rclnodejs.createNode(NODE_NAME);
      PARAMETER = new Parameter(PARAM_NAME, PARAM_TYPE, PARAM_VALUE);
      node.declareParameter(PARAMETER);
    });

    afterEach(function() {
      if (node) node.destroy();
    });

    after(() => {
      rclnodejs.shutdown();
    });

    it('Add and Remove SetParametersCallback', function() {
      const initialCallbackCnt = node._setParametersCallbacks.length;
      const callback1 = () => SET_PARAMETER_RESULT_SUCCESS;
      const callback2 = () => SET_PARAMETER_RESULT_SUCCESS;

      node.addOnSetParametersCallback(callback1);
      node.addOnSetParametersCallback(callback2);

      assert.deepStrictEqual(
        node._setParametersCallbacks.length,
        initialCallbackCnt + 2
      );
      assert.deepStrictEqual(node._setParametersCallbacks[0], callback2);
      assert.deepStrictEqual(node._setParametersCallbacks[1], callback1);

      node.removeOnSetParametersCallback(callback1);
      assert.deepStrictEqual(node._setParametersCallbacks[0], callback2);

      node.removeOnSetParametersCallback(callback2);
      assert.deepStrictEqual(
        initialCallbackCnt,
        node._setParametersCallbacks.length
      );
    });

    it('SetParametersCallback', function() {
      let callbackReceived = false;
      let paramFoundInParamList = false;
      node.addOnSetParametersCallback(parameters => {
        for (const parameter of parameters) {
          if (parameter.name === PARAM_NAME) {
            paramFoundInParamList = true;
            break;
          }
        }
        callbackReceived = true;
        return SET_PARAMETER_RESULT_SUCCESS;
      });

      const param1 = new Parameter(PARAM_NAME, PARAM_TYPE, 'abc');
      node.setParameter(param1);

      assert.ok(node.hasParameter(PARAM_NAME));
      assert.deepStrictEqual(node.getParameter(PARAM_NAME).value, 'abc');
      assert.ok(callbackReceived);
    });

    it('SetParametersCallback reject', function() {
      let callbackReceived = false;
      let paramFoundInParamList = false;
      node.addOnSetParametersCallback(parameters => {
        for (const parameter of parameters) {
          if (parameter.name === PARAM_NAME) {
            paramFoundInParamList = true;
            break;
          }
        }
        callbackReceived = true;
        return SET_PARAMETER_RESULT_FAIL;
      });

      const param1 = new Parameter(PARAM_NAME, PARAM_TYPE, 'abc');
      node.setParameter(param1);

      assert.ok(node.hasParameter(PARAM_NAME));
      assert.deepStrictEqual(node.getParameter(PARAM_NAME).value, PARAM_VALUE);
      assert.ok(callbackReceived);
      assert.ok(paramFoundInParamList);
    });
  });
});
