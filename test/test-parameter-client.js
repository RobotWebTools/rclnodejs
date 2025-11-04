// Copyright (c) 2025 Mahmoud Alghalayini. All rights reserved.
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
const { ParameterType, Parameter, ParameterDescriptor } = rclnodejs;

describe('ParameterClient tests', function () {
  this.timeout(60 * 1000);

  let targetNode;
  let clientNode;
  let paramClient;

  beforeEach(async function () {
    await rclnodejs.init();

    targetNode = rclnodejs.createNode('target_node');

    targetNode.declareParameter(
      new Parameter('string_param', ParameterType.PARAMETER_STRING, 'hello'),
      new ParameterDescriptor(
        'string_param',
        ParameterType.PARAMETER_STRING,
        'A string parameter'
      )
    );

    targetNode.declareParameter(
      new Parameter('int_param', ParameterType.PARAMETER_INTEGER, BigInt(42)),
      new ParameterDescriptor('int_param', ParameterType.PARAMETER_INTEGER)
    );

    targetNode.declareParameter(
      new Parameter('double_param', ParameterType.PARAMETER_DOUBLE, 3.14),
      new ParameterDescriptor('double_param', ParameterType.PARAMETER_DOUBLE)
    );

    targetNode.declareParameter(
      new Parameter('bool_param', ParameterType.PARAMETER_BOOL, true),
      new ParameterDescriptor('bool_param', ParameterType.PARAMETER_BOOL)
    );

    targetNode.declareParameter(
      new Parameter('int_array_param', ParameterType.PARAMETER_INTEGER_ARRAY, [
        BigInt(1),
        BigInt(2),
        BigInt(3),
      ]),
      new ParameterDescriptor(
        'int_array_param',
        ParameterType.PARAMETER_INTEGER_ARRAY
      )
    );

    targetNode.declareParameter(
      new Parameter(
        'double_array_param',
        ParameterType.PARAMETER_DOUBLE_ARRAY,
        [1.1, 2.2, 3.3]
      ),
      new ParameterDescriptor(
        'double_array_param',
        ParameterType.PARAMETER_DOUBLE_ARRAY
      )
    );

    targetNode.declareParameter(
      new Parameter(
        'string_array_param',
        ParameterType.PARAMETER_STRING_ARRAY,
        ['foo', 'bar']
      ),
      new ParameterDescriptor(
        'string_array_param',
        ParameterType.PARAMETER_STRING_ARRAY
      )
    );

    targetNode.declareParameter(
      new Parameter(
        'byte_array_param',
        ParameterType.PARAMETER_BYTE_ARRAY,
        [100, 200, 255]
      ),
      new ParameterDescriptor(
        'byte_array_param',
        ParameterType.PARAMETER_BYTE_ARRAY
      )
    );

    targetNode.declareParameter(
      new Parameter(
        'A.B.nested',
        ParameterType.PARAMETER_STRING,
        'nested_value'
      ),
      new ParameterDescriptor('A.B.nested', ParameterType.PARAMETER_STRING)
    );

    clientNode = rclnodejs.createNode('client_node');

    paramClient = clientNode.createParameterClient('target_node');

    rclnodejs.spin(targetNode);
    rclnodejs.spin(clientNode);

    await paramClient.waitForService(5000);
  });

  afterEach(function () {
    if (paramClient && !paramClient.isDestroyed()) {
      paramClient.destroy();
    }
    if (clientNode) {
      clientNode.destroy();
    }
    if (targetNode) {
      targetNode.destroy();
    }
    rclnodejs.shutdown();
  });

  describe('Constructor and properties', function () {
    it('should create ParameterClient with valid node and name', function () {
      assert.ok(paramClient);
      assert.strictEqual(paramClient.remoteNodeName, 'target_node');
      assert.strictEqual(paramClient.isDestroyed(), false);
    });

    it('should throw error if node is not provided', function () {
      assert.throws(
        () => new rclnodejs.ParameterClient(null, 'test_node'),
        rclnodejs.TypeValidationError
      );
    });

    it('should throw error if remote node name is empty', function () {
      assert.throws(
        () => new rclnodejs.ParameterClient(clientNode, ''),
        rclnodejs.TypeValidationError
      );
    });

    it('should throw error if remote node name is invalid', function () {
      assert.throws(
        () => new rclnodejs.ParameterClient(clientNode, 'invalid@node'),
        Error
      );
    });

    it('should normalize node name by removing leading slash', function () {
      const pc = new rclnodejs.ParameterClient(clientNode, '/some_node');
      assert.strictEqual(pc.remoteNodeName, 'some_node');
      pc.destroy();
    });
  });

  describe('getParameter', function () {
    it('should get string parameter', async function () {
      const param = await paramClient.getParameter('string_param');
      assert.strictEqual(param.name, 'string_param');
      assert.strictEqual(param.type, ParameterType.PARAMETER_STRING);
      assert.strictEqual(param.value, 'hello');
    });

    it('should get integer parameter', async function () {
      const param = await paramClient.getParameter('int_param');
      assert.strictEqual(param.name, 'int_param');
      assert.strictEqual(param.type, ParameterType.PARAMETER_INTEGER);
      assert.strictEqual(param.value, BigInt(42));
    });

    it('should get double parameter', async function () {
      const param = await paramClient.getParameter('double_param');
      assert.strictEqual(param.name, 'double_param');
      assert.strictEqual(param.type, ParameterType.PARAMETER_DOUBLE);
      assert.strictEqual(param.value, 3.14);
    });

    it('should get boolean parameter', async function () {
      const param = await paramClient.getParameter('bool_param');
      assert.strictEqual(param.name, 'bool_param');
      assert.strictEqual(param.type, ParameterType.PARAMETER_BOOL);
      assert.strictEqual(param.value, true);
    });

    it('should get integer array parameter', async function () {
      const param = await paramClient.getParameter('int_array_param');
      assert.strictEqual(param.name, 'int_array_param');
      assert.strictEqual(param.type, ParameterType.PARAMETER_INTEGER_ARRAY);
      assert.deepStrictEqual(param.value, [BigInt(1), BigInt(2), BigInt(3)]);
    });

    it('should get double array parameter', async function () {
      const param = await paramClient.getParameter('double_array_param');
      assert.strictEqual(param.name, 'double_array_param');
      assert.strictEqual(param.type, ParameterType.PARAMETER_DOUBLE_ARRAY);
      assert.deepStrictEqual(param.value, [1.1, 2.2, 3.3]);
    });

    it('should get string array parameter', async function () {
      const param = await paramClient.getParameter('string_array_param');
      assert.strictEqual(param.name, 'string_array_param');
      assert.strictEqual(param.type, ParameterType.PARAMETER_STRING_ARRAY);
      assert.deepStrictEqual(param.value, ['foo', 'bar']);
    });

    it('should get byte array parameter', async function () {
      const param = await paramClient.getParameter('byte_array_param');
      assert.strictEqual(param.name, 'byte_array_param');
      assert.strictEqual(param.type, ParameterType.PARAMETER_BYTE_ARRAY);
      assert.deepStrictEqual(param.value, [100, 200, 255]);
    });

    it('should throw error for non-existent parameter', async function () {
      try {
        await paramClient.getParameter('nonexistent');
        assert.fail('Should have thrown error');
      } catch (error) {
        assert.ok(error.message.includes('not found'));
      }
    });

    it('should throw error if client is destroyed', async function () {
      paramClient.destroy();
      try {
        await paramClient.getParameter('string_param');
        assert.fail('Should have thrown error');
      } catch (error) {
        assert.ok(error.message.includes('destroyed'));
      }
    });
  });

  describe('getParameters', function () {
    it('should get multiple parameters', async function () {
      const params = await paramClient.getParameters([
        'string_param',
        'int_param',
        'bool_param',
      ]);

      assert.strictEqual(params.length, 3);
      assert.strictEqual(params[0].name, 'string_param');
      assert.strictEqual(params[0].value, 'hello');
      assert.strictEqual(params[1].name, 'int_param');
      assert.strictEqual(params[1].value, BigInt(42));
      assert.strictEqual(params[2].name, 'bool_param');
      assert.strictEqual(params[2].value, true);
    });

    it('should return empty array for non-existent parameters', async function () {
      const params = await paramClient.getParameters(['nonexistent']);
      assert.strictEqual(params.length, 0);
    });

    it('should throw error if names is not an array', async function () {
      try {
        await paramClient.getParameters('string_param');
        assert.fail('Should have thrown error');
      } catch (error) {
        assert.ok(error instanceof rclnodejs.TypeValidationError);
        assert.ok(error.message.includes('non-empty array'));
      }
    });

    it('should throw error if names is empty array', async function () {
      try {
        await paramClient.getParameters([]);
        assert.fail('Should have thrown error');
      } catch (error) {
        assert.ok(error instanceof rclnodejs.TypeValidationError);
      }
    });
  });

  describe('setParameter', function () {
    it('should set string parameter', async function () {
      const result = await paramClient.setParameter('string_param', 'world');
      assert.ok(result.successful);
      assert.strictEqual(result.name, 'string_param');

      const param = await paramClient.getParameter('string_param');
      assert.strictEqual(param.value, 'world');
    });

    it('should set integer parameter with automatic BigInt conversion', async function () {
      const result = await paramClient.setParameter('int_param', 100);
      assert.ok(result.successful);

      const param = await paramClient.getParameter('int_param');
      assert.strictEqual(param.value, BigInt(100));
    });

    it('should set double parameter', async function () {
      const result = await paramClient.setParameter('double_param', 2.71);
      assert.ok(result.successful);

      const param = await paramClient.getParameter('double_param');
      assert.strictEqual(param.value, 2.71);
    });

    it('should set boolean parameter', async function () {
      const result = await paramClient.setParameter('bool_param', false);
      assert.ok(result.successful);

      const param = await paramClient.getParameter('bool_param');
      assert.strictEqual(param.value, false);
    });

    it('should set byte array from Uint8Array', async function () {
      const bytes = new Uint8Array([50, 100, 150]);
      const result = await paramClient.setParameter('byte_array_param', bytes);
      assert.ok(result.successful);

      const param = await paramClient.getParameter('byte_array_param');
      assert.deepStrictEqual(param.value, [50, 100, 150]);
    });
  });

  describe('setParameters', function () {
    it('should set multiple parameters', async function () {
      const results = await paramClient.setParameters([
        { name: 'string_param', value: 'test' },
        { name: 'int_param', value: 99 },
        { name: 'bool_param', value: false },
      ]);

      assert.strictEqual(results.length, 3);
      assert.ok(results.every((r) => r.successful));

      const params = await paramClient.getParameters([
        'string_param',
        'int_param',
        'bool_param',
      ]);
      assert.strictEqual(params[0].value, 'test');
      assert.strictEqual(params[1].value, BigInt(99));
      assert.strictEqual(params[2].value, false);
    });

    it('should throw error if parameters is not an array', async function () {
      try {
        await paramClient.setParameters({ name: 'test', value: 'value' });
        assert.fail('Should have thrown error');
      } catch (error) {
        assert.ok(error instanceof rclnodejs.TypeValidationError);
      }
    });

    it('should throw error if parameters is empty array', async function () {
      try {
        await paramClient.setParameters([]);
        assert.fail('Should have thrown error');
      } catch (error) {
        assert.ok(error instanceof rclnodejs.TypeValidationError);
      }
    });
  });

  describe('listParameters', function () {
    it('should list all parameters', async function () {
      const result = await paramClient.listParameters();
      assert.ok(Array.isArray(result.names));
      assert.ok(result.names.length > 0);

      assert.ok(result.names.includes('string_param'));
      assert.ok(result.names.includes('int_param'));
      assert.ok(result.names.includes('bool_param'));
    });

    it('should list parameters with prefix filter', async function () {
      const result = await paramClient.listParameters({
        prefixes: ['A'],
        depth: 10,
      });
      assert.ok(result.names.includes('A.B.nested'));
      assert.ok(Array.isArray(result.prefixes));
    });

    it('should list parameters with depth', async function () {
      const result = await paramClient.listParameters({
        prefixes: ['A'],
        depth: 2,
      });
      assert.ok(Array.isArray(result.names));
      assert.ok(Array.isArray(result.prefixes));
    });
  });

  describe('describeParameters', function () {
    it('should describe single parameter', async function () {
      const descriptors = await paramClient.describeParameters([
        'string_param',
      ]);
      assert.strictEqual(descriptors.length, 1);
      assert.strictEqual(descriptors[0].name, 'string_param');
      assert.strictEqual(descriptors[0].type, ParameterType.PARAMETER_STRING);
      assert.strictEqual(descriptors[0].description, 'A string parameter');
    });

    it('should describe multiple parameters', async function () {
      const descriptors = await paramClient.describeParameters([
        'string_param',
        'int_param',
      ]);
      assert.strictEqual(descriptors.length, 2);
      assert.strictEqual(descriptors[0].name, 'string_param');
      assert.strictEqual(descriptors[1].name, 'int_param');
    });

    it('should throw error if names is not an array', async function () {
      try {
        await paramClient.describeParameters('string_param');
        assert.fail('Should have thrown error');
      } catch (error) {
        assert.ok(error instanceof rclnodejs.TypeValidationError);
      }
    });
  });

  describe('getParameterTypes', function () {
    it('should get types for single parameter', async function () {
      const types = await paramClient.getParameterTypes(['string_param']);
      assert.strictEqual(types.length, 1);
      assert.strictEqual(types[0], ParameterType.PARAMETER_STRING);
    });

    it('should get types for multiple parameters', async function () {
      const types = await paramClient.getParameterTypes([
        'string_param',
        'int_param',
        'bool_param',
      ]);
      assert.strictEqual(types.length, 3);
      assert.strictEqual(types[0], ParameterType.PARAMETER_STRING);
      assert.strictEqual(types[1], ParameterType.PARAMETER_INTEGER);
      assert.strictEqual(types[2], ParameterType.PARAMETER_BOOL);
    });
  });

  describe('Lifecycle management', function () {
    it('should destroy parameter client', function () {
      assert.strictEqual(paramClient.isDestroyed(), false);
      paramClient.destroy();
      assert.strictEqual(paramClient.isDestroyed(), true);
    });

    it('should allow multiple destroy calls', function () {
      paramClient.destroy();
      paramClient.destroy();
      assert.strictEqual(paramClient.isDestroyed(), true);
    });

    it('should auto-destroy when parent node is destroyed', function () {
      const tempNode = rclnodejs.createNode('temp_node');
      const tempClient = tempNode.createParameterClient('target_node');

      assert.strictEqual(tempClient.isDestroyed(), false);
      tempNode.destroy();
      assert.strictEqual(tempClient.isDestroyed(), true);
    });

    it('should destroy via node.destroyParameterClient', function () {
      const tempNode = rclnodejs.createNode('temp_node');
      const tempClient = tempNode.createParameterClient('target_node');

      assert.strictEqual(tempClient.isDestroyed(), false);
      tempNode.destroyParameterClient(tempClient);
      assert.strictEqual(tempClient.isDestroyed(), true);

      tempNode.destroy();
    });
  });

  describe('Timeout and cancellation', function () {
    it('should respect custom timeout option', async function () {
      // This test verifies timeout is passed through
      // Actual timeout behavior is tested in test-async-client.js
      const param = await paramClient.getParameter('string_param', {
        timeout: 10000,
      });
      assert.ok(param);
    });

    it('should support AbortSignal for cancellation', async function () {
      const controller = new AbortController();

      setTimeout(() => controller.abort(), 1);

      try {
        await paramClient.getParameter('string_param', {
          signal: controller.signal,
        });
        assert.fail('Should have been aborted');
      } catch (error) {
        assert.ok(
          error.name === 'AbortError' || error.message.includes('abort')
        );
      }
    });
  });

  describe('Type inference and serialization', function () {
    it('should infer integer type from number', async function () {
      const result = await paramClient.setParameter('int_param', 50);
      assert.ok(result.successful);

      const param = await paramClient.getParameter('int_param');
      assert.strictEqual(param.type, ParameterType.PARAMETER_INTEGER);
      assert.strictEqual(param.value, BigInt(50));
    });

    it('should infer double type from float', async function () {
      const result = await paramClient.setParameter('double_param', 1.23);
      assert.ok(result.successful);

      const param = await paramClient.getParameter('double_param');
      assert.strictEqual(param.type, ParameterType.PARAMETER_DOUBLE);
      assert.strictEqual(param.value, 1.23);
    });

    it('should infer byte array from Uint8Array', async function () {
      const bytes = new Uint8Array([1, 2, 3]);
      const result = await paramClient.setParameter('byte_array_param', bytes);
      assert.ok(result.successful);

      const param = await paramClient.getParameter('byte_array_param');
      assert.strictEqual(param.type, ParameterType.PARAMETER_BYTE_ARRAY);
      assert.deepStrictEqual(param.value, [1, 2, 3]);
    });
  });

  describe('Private field and method encapsulation', function () {
    it('should not allow access to private fields', function () {
      // Private fields should be completely inaccessible
      assert.strictEqual(paramClient['#node'], undefined);
      assert.strictEqual(paramClient['#remoteNodeName'], undefined);
      assert.strictEqual(paramClient['#timeout'], undefined);
      assert.strictEqual(paramClient['#clients'], undefined);
      assert.strictEqual(paramClient['#destroyed'], undefined);
    });

    it('should not allow calling private methods', function () {
      // Private methods should be completely inaccessible
      assert.strictEqual(typeof paramClient['#getOrCreateClient'], 'undefined');
      assert.strictEqual(
        typeof paramClient['#serializeParameterValue'],
        'undefined'
      );
      assert.strictEqual(
        typeof paramClient['#deserializeParameterValue'],
        'undefined'
      );
      assert.strictEqual(typeof paramClient['#normalizeNodeName'], 'undefined');
      assert.strictEqual(typeof paramClient['#toSnakeCase'], 'undefined');
      assert.strictEqual(typeof paramClient['#checkNotDestroyed'], 'undefined');
    });

    it('should only expose public API', function () {
      // Verify only public methods are accessible
      assert.strictEqual(typeof paramClient.getParameter, 'function');
      assert.strictEqual(typeof paramClient.getParameters, 'function');
      assert.strictEqual(typeof paramClient.setParameter, 'function');
      assert.strictEqual(typeof paramClient.setParameters, 'function');
      assert.strictEqual(typeof paramClient.listParameters, 'function');
      assert.strictEqual(typeof paramClient.describeParameters, 'function');
      assert.strictEqual(typeof paramClient.getParameterTypes, 'function');
      assert.strictEqual(typeof paramClient.waitForService, 'function');
      assert.strictEqual(typeof paramClient.isDestroyed, 'function');
      assert.strictEqual(typeof paramClient.destroy, 'function');

      // Public getter
      assert.strictEqual(typeof paramClient.remoteNodeName, 'string');
    });

    it('should have truly private implementation', function () {
      const allProps = Object.getOwnPropertyNames(paramClient);
      const protoProps = Object.getOwnPropertyNames(
        Object.getPrototypeOf(paramClient)
      );

      allProps.forEach((prop) => {
        assert.ok(
          !prop.startsWith('_'),
          `Found underscore property: ${prop} (should use # instead)`
        );
        assert.ok(
          !prop.startsWith('#'),
          `Found # property exposed: ${prop} (should be truly private)`
        );
      });

      protoProps.forEach((prop) => {
        if (prop !== 'constructor') {
          assert.ok(
            !prop.startsWith('_'),
            `Found underscore method: ${prop} (should use # instead)`
          );
          assert.ok(
            !prop.startsWith('#'),
            `Found # method exposed: ${prop} (should be truly private)`
          );
        }
      });
    });
  });
});
