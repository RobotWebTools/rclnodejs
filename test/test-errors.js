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

import assert from 'assert';
import rclnodejs from '../index.js';

describe('Error handling tests', function () {
  describe('RclNodeError', function () {
    it('should create basic RclNodeError', function () {
      const error = new rclnodejs.RclNodeError('Test error');
      assert.ok(error instanceof Error);
      assert.ok(error instanceof rclnodejs.RclNodeError);
      assert.strictEqual(error.name, 'RclNodeError');
      assert.strictEqual(error.message, 'Test error');
      assert.strictEqual(error.code, 'UNKNOWN_ERROR');
      assert.ok(error.timestamp instanceof Date);
    });

    it('should create RclNodeError with options', function () {
      const error = new rclnodejs.RclNodeError('Test error', {
        code: 'TEST_CODE',
        nodeName: 'test_node',
        entityType: 'publisher',
        entityName: 'test_topic',
        details: { key: 'value' },
      });
      assert.strictEqual(error.code, 'TEST_CODE');
      assert.strictEqual(error.nodeName, 'test_node');
      assert.strictEqual(error.entityType, 'publisher');
      assert.strictEqual(error.entityName, 'test_topic');
      assert.deepStrictEqual(error.details, { key: 'value' });
    });

    it('should support error chaining with cause', function () {
      const cause = new Error('Original error');
      const error = new rclnodejs.RclNodeError('Wrapped error', { cause });
      assert.strictEqual(error.cause, cause);
    });

    it('should serialize to JSON', function () {
      const error = new rclnodejs.RclNodeError('Test error', {
        code: 'TEST_CODE',
        nodeName: 'test_node',
      });
      const json = error.toJSON();
      assert.strictEqual(json.name, 'RclNodeError');
      assert.strictEqual(json.message, 'Test error');
      assert.strictEqual(json.code, 'TEST_CODE');
      assert.strictEqual(json.nodeName, 'test_node');
      assert.ok(json.timestamp);
      assert.ok(json.stack);
    });

    it('should have proper toString', function () {
      const error = new rclnodejs.RclNodeError('Test error', {
        code: 'TEST_CODE',
        nodeName: 'test_node',
        entityType: 'publisher',
        entityName: 'test_topic',
      });
      const str = error.toString();
      assert.ok(str.includes('RclNodeError'));
      assert.ok(str.includes('Test error'));
      assert.ok(str.includes('[TEST_CODE]'));
      assert.ok(str.includes('(node: test_node)'));
      assert.ok(str.includes('(publisher: test_topic)'));
    });

    it('should maintain stack trace', function () {
      const error = new rclnodejs.RclNodeError('Test error');
      assert.ok(error.stack);
      assert.ok(error.stack.includes('RclNodeError'));
      assert.ok(error.stack.includes('test-errors.js'));
    });
  });

  describe('ValidationError', function () {
    it('should create ValidationError', function () {
      const error = new rclnodejs.ValidationError('Validation failed');
      assert.ok(error instanceof rclnodejs.RclNodeError);
      assert.ok(error instanceof rclnodejs.ValidationError);
      assert.strictEqual(error.name, 'ValidationError');
      assert.strictEqual(error.code, 'VALIDATION_ERROR');
    });

    it('should include validation details', function () {
      const error = new rclnodejs.ValidationError('Validation failed', {
        argumentName: 'nodeName',
        providedValue: 123,
        expectedType: 'string',
        validationRule: 'type === string',
      });
      assert.strictEqual(error.argumentName, 'nodeName');
      assert.strictEqual(error.providedValue, 123);
      assert.strictEqual(error.expectedType, 'string');
      assert.strictEqual(error.validationRule, 'type === string');
    });
  });

  describe('TypeValidationError', function () {
    it('should create TypeValidationError', function () {
      const error = new rclnodejs.TypeValidationError(
        'nodeName',
        123,
        'string'
      );
      assert.ok(error instanceof rclnodejs.ValidationError);
      assert.ok(error instanceof rclnodejs.TypeValidationError);
      assert.strictEqual(error.name, 'TypeValidationError');
      assert.strictEqual(error.code, 'INVALID_TYPE');
      assert.ok(error.message.includes('nodeName'));
      assert.ok(error.message.includes('string'));
      assert.ok(error.message.includes('number'));
    });

    it('should include correct properties', function () {
      const error = new rclnodejs.TypeValidationError(
        'topicName',
        null,
        'string',
        { nodeName: 'test_node' }
      );
      assert.strictEqual(error.argumentName, 'topicName');
      assert.strictEqual(error.providedValue, null);
      assert.strictEqual(error.expectedType, 'string');
      assert.strictEqual(error.nodeName, 'test_node');
    });
  });

  describe('RangeValidationError', function () {
    it('should create RangeValidationError', function () {
      const error = new rclnodejs.RangeValidationError(
        'frequency',
        1500,
        '0 <= x <= 1000'
      );
      assert.ok(error instanceof rclnodejs.ValidationError);
      assert.ok(error instanceof rclnodejs.RangeValidationError);
      assert.strictEqual(error.name, 'RangeValidationError');
      assert.strictEqual(error.code, 'OUT_OF_RANGE');
      assert.ok(error.message.includes('frequency'));
      assert.ok(error.message.includes('1500'));
      assert.ok(error.message.includes('0 <= x <= 1000'));
    });

    it('should include validation rule', function () {
      const error = new rclnodejs.RangeValidationError(
        'value',
        -5,
        'must be positive'
      );
      assert.strictEqual(error.argumentName, 'value');
      assert.strictEqual(error.providedValue, -5);
      assert.strictEqual(error.validationRule, 'must be positive');
    });
  });

  describe('NameValidationError', function () {
    it('should create NameValidationError', function () {
      const error = new rclnodejs.NameValidationError(
        'my/bad/topic',
        'topic',
        'invalid character',
        5
      );
      assert.ok(error instanceof rclnodejs.ValidationError);
      assert.ok(error instanceof rclnodejs.NameValidationError);
      assert.strictEqual(error.name, 'NameValidationError');
      assert.strictEqual(error.code, 'INVALID_NAME');
      assert.ok(error.message.includes('topic'));
      assert.ok(error.message.includes('my/bad/topic'));
      assert.ok(error.message.includes('invalid character'));
      assert.ok(error.message.includes('at index 5'));
    });

    it('should include validation details', function () {
      const error = new rclnodejs.NameValidationError(
        'bad_node',
        'node',
        'contains underscore',
        3
      );
      assert.strictEqual(error.argumentName, 'node');
      assert.strictEqual(error.providedValue, 'bad_node');
      assert.strictEqual(error.invalidIndex, 3);
      assert.strictEqual(error.validationResult, 'contains underscore');
    });
  });

  describe('OperationError', function () {
    it('should create OperationError', function () {
      const error = new rclnodejs.OperationError('Operation failed');
      assert.ok(error instanceof rclnodejs.RclNodeError);
      assert.ok(error instanceof rclnodejs.OperationError);
      assert.strictEqual(error.name, 'OperationError');
      assert.strictEqual(error.code, 'OPERATION_ERROR');
    });
  });

  describe('TimeoutError', function () {
    it('should create TimeoutError', function () {
      const error = new rclnodejs.TimeoutError('Service request', 5000);
      assert.ok(error instanceof rclnodejs.OperationError);
      assert.ok(error instanceof rclnodejs.TimeoutError);
      assert.strictEqual(error.name, 'TimeoutError');
      assert.strictEqual(error.code, 'TIMEOUT');
      assert.ok(error.message.includes('Service request'));
      assert.ok(error.message.includes('5000ms'));
    });

    it('should include timeout properties', function () {
      const error = new rclnodejs.TimeoutError('Request', 3000, {
        entityName: 'add_two_ints',
      });
      assert.strictEqual(error.timeout, 3000);
      assert.strictEqual(error.operationType, 'Request');
      assert.strictEqual(error.entityName, 'add_two_ints');
    });
  });

  describe('AbortError', function () {
    it('should create AbortError', function () {
      const error = new rclnodejs.AbortError('Service request');
      assert.ok(error instanceof rclnodejs.OperationError);
      assert.ok(error instanceof rclnodejs.AbortError);
      assert.strictEqual(error.name, 'AbortError');
      assert.strictEqual(error.code, 'ABORTED');
      assert.ok(error.message.includes('Service request'));
      assert.ok(error.message.includes('was aborted'));
    });

    it('should create AbortError with reason', function () {
      const error = new rclnodejs.AbortError('Request', 'User cancelled');
      assert.ok(error.message.includes('User cancelled'));
      assert.strictEqual(error.abortReason, 'User cancelled');
    });

    it('should include operation type', function () {
      const error = new rclnodejs.AbortError('Parameter get');
      assert.strictEqual(error.operationType, 'Parameter get');
    });
  });

  describe('ServiceNotFoundError', function () {
    it('should create ServiceNotFoundError', function () {
      const error = new rclnodejs.ServiceNotFoundError('add_two_ints');
      assert.ok(error instanceof rclnodejs.OperationError);
      assert.ok(error instanceof rclnodejs.ServiceNotFoundError);
      assert.strictEqual(error.name, 'ServiceNotFoundError');
      assert.strictEqual(error.code, 'SERVICE_NOT_FOUND');
      assert.ok(error.message.includes('add_two_ints'));
      assert.ok(error.message.includes('not available'));
    });

    it('should include service details', function () {
      const error = new rclnodejs.ServiceNotFoundError('my_service', {
        nodeName: 'client_node',
      });
      assert.strictEqual(error.serviceName, 'my_service');
      assert.strictEqual(error.entityType, 'service');
      assert.strictEqual(error.entityName, 'my_service');
      assert.strictEqual(error.nodeName, 'client_node');
    });
  });

  describe('NodeNotFoundError', function () {
    it('should create NodeNotFoundError', function () {
      const error = new rclnodejs.NodeNotFoundError('remote_node');
      assert.ok(error instanceof rclnodejs.OperationError);
      assert.ok(error instanceof rclnodejs.NodeNotFoundError);
      assert.strictEqual(error.name, 'NodeNotFoundError');
      assert.strictEqual(error.code, 'NODE_NOT_FOUND');
      assert.ok(error.message.includes('remote_node'));
      assert.ok(error.message.includes('not found'));
    });

    it('should include node details', function () {
      const error = new rclnodejs.NodeNotFoundError('target_node');
      assert.strictEqual(error.targetNodeName, 'target_node');
      assert.strictEqual(error.entityType, 'node');
      assert.strictEqual(error.entityName, 'target_node');
    });
  });

  describe('ParameterError', function () {
    it('should create ParameterError', function () {
      const error = new rclnodejs.ParameterError(
        'Parameter operation failed',
        'max_speed'
      );
      assert.ok(error instanceof rclnodejs.RclNodeError);
      assert.ok(error instanceof rclnodejs.ParameterError);
      assert.strictEqual(error.name, 'ParameterError');
      assert.strictEqual(error.code, 'PARAMETER_ERROR');
      assert.strictEqual(error.parameterName, 'max_speed');
      assert.strictEqual(error.entityType, 'parameter');
      assert.strictEqual(error.entityName, 'max_speed');
    });
  });

  describe('ParameterNotFoundError', function () {
    it('should create ParameterNotFoundError', function () {
      const error = new rclnodejs.ParameterNotFoundError(
        'max_speed',
        'robot_node'
      );
      assert.ok(error instanceof rclnodejs.ParameterError);
      assert.ok(error instanceof rclnodejs.ParameterNotFoundError);
      assert.strictEqual(error.name, 'ParameterNotFoundError');
      assert.strictEqual(error.code, 'PARAMETER_NOT_FOUND');
      assert.ok(error.message.includes('max_speed'));
      assert.ok(error.message.includes('robot_node'));
      assert.strictEqual(error.parameterName, 'max_speed');
      assert.strictEqual(error.nodeName, 'robot_node');
    });
  });

  describe('ParameterTypeError', function () {
    it('should create ParameterTypeError', function () {
      const error = new rclnodejs.ParameterTypeError(
        'max_speed',
        'PARAMETER_DOUBLE',
        'PARAMETER_INTEGER'
      );
      assert.ok(error instanceof rclnodejs.ParameterError);
      assert.ok(error instanceof rclnodejs.ParameterTypeError);
      assert.strictEqual(error.name, 'ParameterTypeError');
      assert.strictEqual(error.code, 'PARAMETER_TYPE_MISMATCH');
      assert.ok(error.message.includes('max_speed'));
      assert.ok(error.message.includes('PARAMETER_DOUBLE'));
      assert.ok(error.message.includes('PARAMETER_INTEGER'));
    });

    it('should include type details', function () {
      const error = new rclnodejs.ParameterTypeError('param', 'int', 'string');
      assert.strictEqual(error.expectedType, 'int');
      assert.strictEqual(error.actualType, 'string');
    });
  });

  describe('ReadOnlyParameterError', function () {
    it('should create ReadOnlyParameterError', function () {
      const error = new rclnodejs.ReadOnlyParameterError('use_sim_time');
      assert.ok(error instanceof rclnodejs.ParameterError);
      assert.ok(error instanceof rclnodejs.ReadOnlyParameterError);
      assert.strictEqual(error.name, 'ReadOnlyParameterError');
      assert.strictEqual(error.code, 'PARAMETER_READ_ONLY');
      assert.ok(error.message.includes('use_sim_time'));
      assert.ok(error.message.includes('read-only'));
      assert.strictEqual(error.parameterName, 'use_sim_time');
    });
  });

  describe('TopicError', function () {
    it('should create TopicError', function () {
      const error = new rclnodejs.TopicError(
        'Topic operation failed',
        '/chatter'
      );
      assert.ok(error instanceof rclnodejs.RclNodeError);
      assert.ok(error instanceof rclnodejs.TopicError);
      assert.strictEqual(error.name, 'TopicError');
      assert.strictEqual(error.code, 'TOPIC_ERROR');
      assert.strictEqual(error.topicName, '/chatter');
      assert.strictEqual(error.entityType, 'topic');
      assert.strictEqual(error.entityName, '/chatter');
    });
  });

  describe('PublisherError', function () {
    it('should create PublisherError', function () {
      const error = new rclnodejs.PublisherError(
        'Failed to publish',
        '/chatter'
      );
      assert.ok(error instanceof rclnodejs.TopicError);
      assert.ok(error instanceof rclnodejs.PublisherError);
      assert.strictEqual(error.name, 'PublisherError');
      assert.strictEqual(error.code, 'PUBLISHER_ERROR');
      assert.strictEqual(error.entityType, 'publisher');
    });
  });

  describe('SubscriptionError', function () {
    it('should create SubscriptionError', function () {
      const error = new rclnodejs.SubscriptionError(
        'Failed to subscribe',
        '/chatter'
      );
      assert.ok(error instanceof rclnodejs.TopicError);
      assert.ok(error instanceof rclnodejs.SubscriptionError);
      assert.strictEqual(error.name, 'SubscriptionError');
      assert.strictEqual(error.code, 'SUBSCRIPTION_ERROR');
      assert.strictEqual(error.entityType, 'subscription');
    });
  });

  describe('ActionError', function () {
    it('should create ActionError', function () {
      const error = new rclnodejs.ActionError(
        'Action operation failed',
        'fibonacci'
      );
      assert.ok(error instanceof rclnodejs.RclNodeError);
      assert.ok(error instanceof rclnodejs.ActionError);
      assert.strictEqual(error.name, 'ActionError');
      assert.strictEqual(error.code, 'ACTION_ERROR');
      assert.strictEqual(error.actionName, 'fibonacci');
      assert.strictEqual(error.entityType, 'action');
      assert.strictEqual(error.entityName, 'fibonacci');
    });
  });

  describe('GoalRejectedError', function () {
    it('should create GoalRejectedError', function () {
      const error = new rclnodejs.GoalRejectedError(
        'fibonacci',
        'goal-123-abc'
      );
      assert.ok(error instanceof rclnodejs.ActionError);
      assert.ok(error instanceof rclnodejs.GoalRejectedError);
      assert.strictEqual(error.name, 'GoalRejectedError');
      assert.strictEqual(error.code, 'GOAL_REJECTED');
      assert.ok(error.message.includes('fibonacci'));
      assert.strictEqual(error.goalId, 'goal-123-abc');
    });
  });

  describe('ActionServerNotFoundError', function () {
    it('should create ActionServerNotFoundError', function () {
      const error = new rclnodejs.ActionServerNotFoundError('fibonacci');
      assert.ok(error instanceof rclnodejs.ActionError);
      assert.ok(error instanceof rclnodejs.ActionServerNotFoundError);
      assert.strictEqual(error.name, 'ActionServerNotFoundError');
      assert.strictEqual(error.code, 'ACTION_SERVER_NOT_FOUND');
      assert.ok(error.message.includes('fibonacci'));
      assert.ok(error.message.includes('not available'));
    });
  });

  describe('NativeError', function () {
    it('should create NativeError', function () {
      const error = new rclnodejs.NativeError(
        'rcl_take failed',
        'subscription receive'
      );
      assert.ok(error instanceof rclnodejs.RclNodeError);
      assert.ok(error instanceof rclnodejs.NativeError);
      assert.strictEqual(error.name, 'NativeError');
      assert.strictEqual(error.code, 'NATIVE_ERROR');
      assert.ok(error.message.includes('Native operation failed'));
      assert.ok(error.message.includes('subscription receive'));
      assert.ok(error.message.includes('rcl_take failed'));
    });

    it('should include native details', function () {
      const error = new rclnodejs.NativeError('invalid handle', 'publish', {
        entityName: '/chatter',
      });
      assert.strictEqual(error.nativeMessage, 'invalid handle');
      assert.strictEqual(error.operation, 'publish');
      assert.strictEqual(error.entityName, '/chatter');
    });
  });

  describe('Error chaining', function () {
    it('should preserve cause chain', function () {
      const original = new Error('Original error');
      const wrapped = new rclnodejs.RclNodeError('Wrapped', {
        cause: original,
      });
      const final = new rclnodejs.TimeoutError('Request', 5000, {
        cause: wrapped,
      });

      assert.strictEqual(final.cause, wrapped);
      assert.strictEqual(wrapped.cause, original);
    });

    it('should include cause in JSON', function () {
      const cause = new Error('Original error');
      const error = new rclnodejs.RclNodeError('Wrapped', { cause });
      const json = error.toJSON();
      assert.ok(json.cause);
      assert.strictEqual(json.cause, 'Original error');
    });

    it('should handle RclNodeError cause in JSON', function () {
      const cause = new rclnodejs.ValidationError('Validation failed');
      const error = new rclnodejs.OperationError('Operation failed', {
        cause,
      });
      const json = error.toJSON();
      assert.ok(json.cause);
      assert.strictEqual(json.cause.name, 'ValidationError');
    });
  });

  describe('Error inheritance', function () {
    it('should maintain proper inheritance chain', function () {
      const error = new rclnodejs.TypeValidationError('arg', 123, 'string');
      assert.ok(error instanceof Error);
      assert.ok(error instanceof rclnodejs.RclNodeError);
      assert.ok(error instanceof rclnodejs.ValidationError);
      assert.ok(error instanceof rclnodejs.TypeValidationError);
    });

    it('should work with instanceof checks', function () {
      const errors = [
        new rclnodejs.TimeoutError('Test', 1000),
        new rclnodejs.ParameterNotFoundError('param', 'node'),
        new rclnodejs.ServiceNotFoundError('service'),
        new rclnodejs.PublisherError('msg', 'topic'),
      ];

      errors.forEach((err) => {
        assert.ok(err instanceof Error);
        assert.ok(err instanceof rclnodejs.RclNodeError);
      });
    });
  });

  describe('Error catching patterns', function () {
    it('should catch specific error types', function () {
      function throwTimeout() {
        throw new rclnodejs.TimeoutError('Request', 5000);
      }

      let caught = false;
      try {
        throwTimeout();
      } catch (error) {
        if (error instanceof rclnodejs.TimeoutError) {
          caught = true;
          assert.strictEqual(error.timeout, 5000);
        }
      }
      assert.ok(caught);
    });

    it('should catch base error types', function () {
      function throwValidationError() {
        throw new rclnodejs.TypeValidationError('arg', 123, 'string');
      }

      let caught = false;
      try {
        throwValidationError();
      } catch (error) {
        if (error instanceof rclnodejs.ValidationError) {
          caught = true;
        }
      }
      assert.ok(caught);
    });
  });
});
