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
const { isClose } = require('../lib/utils.js');
const rclnodejs = require('../index.js');
const loader = require('../lib/interface_loader.js');

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

describe('rclnodejs parameters test suite', function () {
  describe('parameter api tests', function () {
    it('Parameter constructor', function () {
      let param = new Parameter(
        'str_param',
        ParameterType.PARAMETER_STRING,
        'foobar'
      );

      assert.equal(param.name, 'str_param');
      assert.equal(param.type, ParameterType.PARAMETER_STRING);
      assert.equal(param.value, 'foobar');
    });

    it('Parameter update', function () {
      let param = new Parameter(
        'int_param',
        ParameterType.PARAMETER_INTEGER,
        100n
      );
      assert.strictEqual(param.value, 100n);

      param.value = 101n;
      assert.strictEqual(param.value, 101n);

      assertThrowsError(
        () => (param.value = 'hello world'),
        rclnodejs.ParameterTypeError
      );
    });
  });

  describe('convert fromParameterMessage test suite', function () {
    it('should convert PARAMETER_NOT_SET type', function () {
      let param_msg = new (loader.loadInterface(
        'rcl_interfaces/msg/Parameter'
      ))();
      param_msg.name = 'not_set_param';
      param_msg.value.type = ParameterType.PARAMETER_NOT_SET;

      let param;
      assert.doesNotThrow(() => {
        param = Parameter.fromParameterMessage(param_msg);
      });
      assert.strictEqual(param.name, 'not_set_param');
      assert.strictEqual(param.type, ParameterType.PARAMETER_NOT_SET);
      assert.strictEqual(param.value, undefined);
    });

    it('should convert PARAMETER_BOOL type', function () {
      let param_msg = new (loader.loadInterface(
        'rcl_interfaces/msg/Parameter'
      ))();
      param_msg.name = 'bool_param';
      param_msg.value.type = ParameterType.PARAMETER_BOOL;
      param_msg.value.bool_value = false;

      let param;
      assert.doesNotThrow(() => {
        param = Parameter.fromParameterMessage(param_msg);
      });
      assert.strictEqual(param.name, 'bool_param');
      assert.strictEqual(param.type, ParameterType.PARAMETER_BOOL);
      assert.strictEqual(param.value, false);
    });

    it('should convert PARAMETER_BOOL_ARRAY type', function () {
      let param_msg = new (loader.loadInterface(
        'rcl_interfaces/msg/Parameter'
      ))();
      param_msg.name = 'bool_array_param';
      param_msg.value.type = ParameterType.PARAMETER_BOOL_ARRAY;
      param_msg.value.bool_array_value = [true, false, true];

      let param;
      assert.doesNotThrow(() => {
        param = Parameter.fromParameterMessage(param_msg);
      });
      assert.strictEqual(param.name, 'bool_array_param');
      assert.strictEqual(param.type, ParameterType.PARAMETER_BOOL_ARRAY);
      assert.deepStrictEqual(param.value, [true, false, true]);
    });

    it('should convert PARAMETER_BYTE_ARRAY type', function () {
      let param_msg = new (loader.loadInterface(
        'rcl_interfaces/msg/Parameter'
      ))();
      param_msg.name = 'byte_array_param';
      param_msg.value.type = ParameterType.PARAMETER_BYTE_ARRAY;
      param_msg.value.byte_array_value = [1, 2, 3];

      let param;
      assert.doesNotThrow(() => {
        param = Parameter.fromParameterMessage(param_msg);
      });
      assert.strictEqual(param.name, 'byte_array_param');
      assert.strictEqual(param.type, ParameterType.PARAMETER_BYTE_ARRAY);
      assert.deepStrictEqual(param.value, [1, 2, 3]);
    });

    it('should convert PARAMETER_DOUBLE type', function () {
      let param_msg = new (loader.loadInterface(
        'rcl_interfaces/msg/Parameter'
      ))();
      param_msg.name = 'double_param';
      param_msg.value.type = ParameterType.PARAMETER_DOUBLE;
      param_msg.value.double_value = 1.23;

      let param;
      assert.doesNotThrow(() => {
        param = Parameter.fromParameterMessage(param_msg);
      });
      assert.strictEqual(param.name, 'double_param');
      assert.strictEqual(param.type, ParameterType.PARAMETER_DOUBLE);
      assert.strictEqual(param.value, 1.23);
    });

    it('should convert PARAMETER_DOUBLE_ARRAY type', function () {
      let param_msg = new (loader.loadInterface(
        'rcl_interfaces/msg/Parameter'
      ))();
      param_msg.name = 'double_array_param';
      param_msg.value.type = ParameterType.PARAMETER_DOUBLE_ARRAY;
      param_msg.value.double_array_value = [1.1, 2.2, 3.3, 4.4, 5.5];

      let param;
      assert.doesNotThrow(() => {
        param = Parameter.fromParameterMessage(param_msg);
      });
      assert.strictEqual(param.name, 'double_array_param');
      assert.strictEqual(param.type, ParameterType.PARAMETER_DOUBLE_ARRAY);
      assert.deepStrictEqual(param.value, [1.1, 2.2, 3.3, 4.4, 5.5]);
    });

    it('should convert PARAMETER_INTEGER type', function () {
      let param_msg = new (loader.loadInterface(
        'rcl_interfaces/msg/Parameter'
      ))();
      param_msg.name = 'integer_param';
      param_msg.value.type = ParameterType.PARAMETER_INTEGER;
      param_msg.value.integer_value = 123n;

      let param;
      assert.doesNotThrow(() => {
        param = Parameter.fromParameterMessage(param_msg);
      });
      assert.strictEqual(param.name, 'integer_param');
      assert.strictEqual(param.type, ParameterType.PARAMETER_INTEGER);
      assert.strictEqual(param.value, 123n);
    });

    it('should convert PARAMETER_INTEGER_ARRAY type', function () {
      let param_msg = new (loader.loadInterface(
        'rcl_interfaces/msg/Parameter'
      ))();
      param_msg.name = 'integer_array_param';
      param_msg.value.type = ParameterType.PARAMETER_INTEGER_ARRAY;
      param_msg.value.integer_array_value = [1n, 2n, 3n, 4n, 5n];

      let param;
      assert.doesNotThrow(() => {
        param = Parameter.fromParameterMessage(param_msg);
      });
      assert.strictEqual(param.name, 'integer_array_param');
      assert.strictEqual(param.type, ParameterType.PARAMETER_INTEGER_ARRAY);
      assert.deepStrictEqual(param.value, [1n, 2n, 3n, 4n, 5n]);
    });

    it('should convert PARAMETER_STRING type', function () {
      let param_msg = new (loader.loadInterface(
        'rcl_interfaces/msg/Parameter'
      ))();
      param_msg.name = 'string_param';
      param_msg.value.type = ParameterType.PARAMETER_STRING;
      param_msg.value.string_value = 'hello world';

      let param;
      assert.doesNotThrow(() => {
        param = Parameter.fromParameterMessage(param_msg);
      });
      assert.strictEqual(param.name, 'string_param');
      assert.strictEqual(param.type, ParameterType.PARAMETER_STRING);
      assert.strictEqual(param.value, 'hello world');
    });

    it('should convert PARAMETER_STRING_ARRAY type', function () {
      let param_msg = new (loader.loadInterface(
        'rcl_interfaces/msg/Parameter'
      ))();
      param_msg.name = 'string_array_param';
      param_msg.value.type = ParameterType.PARAMETER_STRING_ARRAY;
      param_msg.value.string_array_value = ['hello', 'world'];

      let param;
      assert.doesNotThrow(() => {
        param = Parameter.fromParameterMessage(param_msg);
      });
      assert.strictEqual(param.name, 'string_array_param');
      assert.strictEqual(param.type, ParameterType.PARAMETER_STRING_ARRAY);
      assert.deepStrictEqual(param.value, ['hello', 'world']);
    });
  });

  describe('range tests', function () {
    it('Math IsClose test', function () {
      assert.ok(isClose(1.0, 1.0, DEFAULT_NUMERIC_RANGE_TOLERANCE));
      assert.ok(!isClose(1.0, 1.1, DEFAULT_NUMERIC_RANGE_TOLERANCE));
      assert.ok(isClose(1.0, 1.1, 0.11));
    });

    it('IntegerRange test', function () {
      const range = new IntegerRange(0n, 100n, 1n);

      // test property accessors
      assert.equal(range.fromValue, 0n);
      assert.equal(range.toValue, 100n);
      assert.equal(range.step, 1n);

      // test midpoint
      assert.ok(range.inRange(50n));

      // test lower boundary
      assert.ok(range.inRange(1n));
      assert.ok(range.inRange(0n));
      assert.ok(!range.inRange(-1n));

      // test upper boundary
      assert.ok(range.inRange(99n));
      assert.ok(range.inRange(100n));
      assert.ok(!range.inRange(101n));
    });

    it('FloatingPointRange test', function () {
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

  describe('parameter-descriptor tests', function () {
    it('String descriptor test', function () {
      assert.equal('strD', STRING_DESCRIPTOR.name);
      assert.equal(ParameterType.PARAMETER_STRING, STRING_DESCRIPTOR.type);
      assert.equal('no description', STRING_DESCRIPTOR.description);
      assert.ok(!STRING_DESCRIPTOR.readOnly);

      assert.equal('strDRO', STRING_READONLY_DESCRIPTOR.name);
      assert.equal('basic description', STRING_READONLY_DESCRIPTOR.description);
      assert.ok(STRING_READONLY_DESCRIPTOR.readOnly);
    });

    it('Integer descriptor with [0-255] range test', function () {
      const descriptor = new ParameterDescriptor(
        'int_param',
        ParameterType.PARAMETER_INTEGER
      );
      descriptor.range = new IntegerRange(0n, 255n);

      const param = new Parameter(
        'int_param',
        ParameterType.PARAMETER_INTEGER,
        100n
      );
      assert.ifError(descriptor.validateParameter(param));

      param.value = 255n;
      assert.ifError(descriptor.validateParameter(param));

      param.value = -1n;
      assertThrowsError(
        () => descriptor.validateParameter(param),
        rclnodejs.RangeValidationError
      );

      param.value = 256n;
      assertThrowsError(
        () => descriptor.validateParameter(param),
        rclnodejs.RangeValidationError
      );
    });

    it('Integer descriptor with [0-255], step=5 range test', function () {
      const descriptor = new ParameterDescriptor(
        'int_param',
        ParameterType.PARAMETER_INTEGER
      );
      descriptor.range = new IntegerRange(0n, 255n, 5n);

      const param = new Parameter(
        'int_param',
        ParameterType.PARAMETER_INTEGER,
        100n
      );
      assert.ifError(descriptor.validateParameter(param));

      param.value = 255n;
      assert.ifError(descriptor.validateParameter(param));

      param.value = 1n;
      assertThrowsError(
        () => descriptor.validateParameter(param),
        rclnodejs.RangeValidationError
      );

      param.value = 256n;
      assertThrowsError(
        () => descriptor.validateParameter(param),
        rclnodejs.RangeValidationError
      );
    });
  });

  describe('rcl parameter-overrides test', function () {
    const NODE_NAME = 'test_node';

    let node;
    this.timeout(60 * 1000);

    afterEach(function () {
      if (node) node.destroy();
      rclnodejs.shutdown();
    });

    it('cli parameter-override: 1 global param', async function () {
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

    it('cli parameter-override: 1 global param redefined', async function () {
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

    it('cli parameter override, global & node:param', async function () {
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

    it('cli parameter override, filter out other node parameters', async function () {
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

    it('cli + constructor parameter overrides', async function () {
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

    it('cli load parameter.yaml file ', async function () {
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

    it('cli load from parameter.yaml file with global /** param group ', async function () {
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

    it('override param from parameter.yaml file ', async function () {
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

  describe('parameter onSetParameterCallback tests', function () {
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

    before(async () => {
      await rclnodejs.init();
    });

    this.beforeEach(async () => {
      node = rclnodejs.createNode(NODE_NAME);
      PARAMETER = new Parameter(PARAM_NAME, PARAM_TYPE, PARAM_VALUE);
      node.declareParameter(PARAMETER);
    });

    afterEach(function () {
      if (node) node.destroy();
    });

    after(() => {
      rclnodejs.shutdown();
    });

    it('Add and Remove SetParametersCallback', function () {
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

    it('SetParametersCallback', function () {
      let callbackReceived = false;
      let paramFoundInParamList = false;
      node.addOnSetParametersCallback((parameters) => {
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

    it('SetParametersCallback reject', function () {
      let callbackReceived = false;
      let paramFoundInParamList = false;
      node.addOnSetParametersCallback((parameters) => {
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

  describe('parameterOverrides validation', function () {
    const NODE_NAME = 'test_node';
    let node;
    this.timeout(60 * 1000);

    afterEach(function () {
      if (node) node.destroy();
      rclnodejs.shutdown();
    });

    it('should reject non-Parameter objects in parameterOverrides', async function () {
      await rclnodejs.init();
      const options = new NodeOptions();
      options.parameterOverrides = [{ name: 'p1', value: 'not a Parameter' }];
      assert.throws(
        () =>
          rclnodejs.createNode(
            NODE_NAME,
            '',
            Context.defaultContext(),
            options
          ),
        (err) => err instanceof rclnodejs.TypeValidationError,
        'Expected TypeValidationError for non-Parameter parameterOverride'
      );
    });
  });

  describe('parameter type validation', function () {
    it('validType should reject invalid numeric types', function () {
      // A Parameter with an out-of-range type should throw on validate()
      assert.throws(() => {
        new Parameter('bad', 999, 'value').validate();
      });
    });

    it('validType should accept all known ParameterTypes', function () {
      // NOT_SET type
      assert.doesNotThrow(() => {
        new Parameter('p', ParameterType.PARAMETER_NOT_SET).validate();
      });
      // BOOL type
      assert.doesNotThrow(() => {
        new Parameter('p', ParameterType.PARAMETER_BOOL, true).validate();
      });
      // STRING type
      assert.doesNotThrow(() => {
        new Parameter('p', ParameterType.PARAMETER_STRING, 'hello').validate();
      });
    });

    it('array parameter validation', function () {
      assert.doesNotThrow(() => {
        new Parameter('p', ParameterType.PARAMETER_STRING_ARRAY, [
          'a',
          'b',
        ]).validate();
      });
      assert.doesNotThrow(() => {
        new Parameter('p', ParameterType.PARAMETER_INTEGER_ARRAY, [
          1n,
          2n,
        ]).validate();
      });
      assert.doesNotThrow(() => {
        new Parameter(
          'p',
          ParameterType.PARAMETER_DOUBLE_ARRAY,
          [1.0, 2.0]
        ).validate();
      });
      assert.doesNotThrow(() => {
        new Parameter('p', ParameterType.PARAMETER_BOOL_ARRAY, [
          true,
          false,
        ]).validate();
      });
    });
  });
});
