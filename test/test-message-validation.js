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

describe('Message Validation Tests', function () {
  this.timeout(60 * 1000);

  before(async function () {
    await rclnodejs.init();
  });

  after(function () {
    rclnodejs.shutdown();
  });

  describe('ValidationProblem enum', function () {
    it('should have all problem types defined', function () {
      assert.strictEqual(
        rclnodejs.ValidationProblem.UNKNOWN_FIELD,
        'UNKNOWN_FIELD'
      );
      assert.strictEqual(
        rclnodejs.ValidationProblem.TYPE_MISMATCH,
        'TYPE_MISMATCH'
      );
      assert.strictEqual(
        rclnodejs.ValidationProblem.MISSING_FIELD,
        'MISSING_FIELD'
      );
      assert.strictEqual(
        rclnodejs.ValidationProblem.ARRAY_LENGTH,
        'ARRAY_LENGTH'
      );
      assert.strictEqual(
        rclnodejs.ValidationProblem.OUT_OF_RANGE,
        'OUT_OF_RANGE'
      );
      assert.strictEqual(
        rclnodejs.ValidationProblem.NESTED_ERROR,
        'NESTED_ERROR'
      );
    });
  });

  describe('getMessageSchema', function () {
    it('should return schema for std_msgs/msg/String', function () {
      const StringClass = rclnodejs.require('std_msgs/msg/String');
      const schema = rclnodejs.getMessageSchema(StringClass);

      assert.ok(schema);
      assert.strictEqual(schema.messageType, 'std_msgs/msg/String');
      assert.ok(Array.isArray(schema.fields));
      assert.ok(schema.fields.length > 0);

      const dataField = schema.fields.find((f) => f.name === 'data');
      assert.ok(dataField);
      assert.strictEqual(dataField.type.type, 'string');
      assert.strictEqual(dataField.type.isPrimitiveType, true);
    });

    it('should return schema for geometry_msgs/msg/Twist', function () {
      const TwistClass = rclnodejs.require('geometry_msgs/msg/Twist');
      const schema = rclnodejs.getMessageSchema(TwistClass);

      assert.ok(schema);
      assert.strictEqual(schema.messageType, 'geometry_msgs/msg/Twist');
      assert.ok(schema.fields.length === 2);

      const linearField = schema.fields.find((f) => f.name === 'linear');
      assert.ok(linearField);
      assert.strictEqual(linearField.type.isPrimitiveType, false);
      assert.strictEqual(linearField.type.type, 'Vector3');
    });

    it('should return null for invalid type class', function () {
      const schema = rclnodejs.getMessageSchema({});
      assert.strictEqual(schema, null);
    });
  });

  describe('getFieldNames', function () {
    it('should return field names for std_msgs/msg/String', function () {
      const StringClass = rclnodejs.require('std_msgs/msg/String');
      const fields = rclnodejs.getFieldNames(StringClass);

      assert.ok(Array.isArray(fields));
      assert.ok(fields.includes('data'));
    });

    it('should return field names for geometry_msgs/msg/Twist', function () {
      const TwistClass = rclnodejs.require('geometry_msgs/msg/Twist');
      const fields = rclnodejs.getFieldNames(TwistClass);

      assert.ok(Array.isArray(fields));
      assert.ok(fields.includes('linear'));
      assert.ok(fields.includes('angular'));
    });
  });

  describe('getFieldType', function () {
    it('should return field type for std_msgs/msg/String.data', function () {
      const StringClass = rclnodejs.require('std_msgs/msg/String');
      const fieldType = rclnodejs.getFieldType(StringClass, 'data');

      assert.ok(fieldType);
      assert.strictEqual(fieldType.type, 'string');
      assert.strictEqual(fieldType.isPrimitiveType, true);
    });

    it('should return null for unknown field', function () {
      const StringClass = rclnodejs.require('std_msgs/msg/String');
      const fieldType = rclnodejs.getFieldType(StringClass, 'unknown_field');

      assert.strictEqual(fieldType, null);
    });
  });

  describe('validateMessage', function () {
    it('should validate correct std_msgs/msg/String', function () {
      const StringClass = rclnodejs.require('std_msgs/msg/String');
      const result = rclnodejs.validateMessage({ data: 'hello' }, StringClass);

      assert.strictEqual(result.valid, true);
      assert.strictEqual(result.issues.length, 0);
    });

    it('should validate primitive value for wrapper message', function () {
      const StringClass = rclnodejs.require('std_msgs/msg/String');
      const result = rclnodejs.validateMessage('hello', StringClass);

      assert.strictEqual(result.valid, true);
      assert.strictEqual(result.issues.length, 0);
    });

    it('should detect type mismatch for std_msgs/msg/String', function () {
      const StringClass = rclnodejs.require('std_msgs/msg/String');
      const result = rclnodejs.validateMessage({ data: 123 }, StringClass);

      assert.strictEqual(result.valid, false);
      assert.ok(result.issues.length > 0);

      const typeIssue = result.issues.find(
        (i) => i.problem === 'TYPE_MISMATCH'
      );
      assert.ok(typeIssue);
      assert.strictEqual(typeIssue.field, 'data');
      assert.strictEqual(typeIssue.expected, 'string');
      assert.strictEqual(typeIssue.received, 'number');
    });

    it('should detect unknown fields in strict mode', function () {
      const StringClass = rclnodejs.require('std_msgs/msg/String');
      const result = rclnodejs.validateMessage(
        { data: 'hello', unknown_field: 'value' },
        StringClass,
        { strict: true }
      );

      assert.strictEqual(result.valid, false);
      const unknownIssue = result.issues.find(
        (i) => i.problem === 'UNKNOWN_FIELD'
      );
      assert.ok(unknownIssue);
      assert.strictEqual(unknownIssue.field, 'unknown_field');
    });

    it('should allow unknown fields in non-strict mode', function () {
      const StringClass = rclnodejs.require('std_msgs/msg/String');
      const result = rclnodejs.validateMessage(
        { data: 'hello', unknown_field: 'value' },
        StringClass,
        { strict: false }
      );

      assert.strictEqual(result.valid, true);
    });

    it('should validate nested messages', function () {
      const TwistClass = rclnodejs.require('geometry_msgs/msg/Twist');
      const result = rclnodejs.validateMessage(
        {
          linear: { x: 1.0, y: 2.0, z: 3.0 },
          angular: { x: 0.0, y: 0.0, z: 0.5 },
        },
        TwistClass
      );

      assert.strictEqual(result.valid, true);
    });

    it('should detect type mismatch in nested messages', function () {
      const TwistClass = rclnodejs.require('geometry_msgs/msg/Twist');
      const result = rclnodejs.validateMessage(
        {
          linear: { x: 'not a number', y: 2.0, z: 3.0 },
          angular: { x: 0.0, y: 0.0, z: 0.5 },
        },
        TwistClass
      );

      assert.strictEqual(result.valid, false);
      const typeIssue = result.issues.find(
        (i) => i.problem === 'TYPE_MISMATCH'
      );
      assert.ok(typeIssue);
      assert.ok(typeIssue.field.includes('linear'));
    });

    it('should handle null input', function () {
      const StringClass = rclnodejs.require('std_msgs/msg/String');
      const result = rclnodejs.validateMessage(null, StringClass);

      assert.strictEqual(result.valid, false);
    });

    it('should handle undefined input', function () {
      const StringClass = rclnodejs.require('std_msgs/msg/String');
      const result = rclnodejs.validateMessage(undefined, StringClass);

      assert.strictEqual(result.valid, false);
    });
  });

  describe('assertValidMessage', function () {
    it('should not throw for valid message', function () {
      const StringClass = rclnodejs.require('std_msgs/msg/String');

      assert.doesNotThrow(() => {
        rclnodejs.assertValidMessage({ data: 'hello' }, StringClass);
      });
    });

    it('should throw MessageValidationError for invalid message', function () {
      const StringClass = rclnodejs.require('std_msgs/msg/String');

      assert.throws(
        () => {
          rclnodejs.assertValidMessage({ data: 123 }, StringClass);
        },
        (error) => {
          assert.ok(error instanceof rclnodejs.MessageValidationError);
          assert.strictEqual(error.messageType, 'std_msgs/msg/String');
          assert.ok(Array.isArray(error.issues));
          assert.ok(error.issues.length > 0);
          return true;
        }
      );
    });

    it('should throw for unknown fields in strict mode', function () {
      const StringClass = rclnodejs.require('std_msgs/msg/String');

      assert.throws(() => {
        rclnodejs.assertValidMessage(
          { data: 'hello', unknown: 'field' },
          StringClass,
          { strict: true }
        );
      }, rclnodejs.MessageValidationError);
    });
  });

  describe('createMessageValidator', function () {
    it('should create a reusable validator', function () {
      const StringClass = rclnodejs.require('std_msgs/msg/String');
      const validator = rclnodejs.createMessageValidator(StringClass, {
        strict: true,
      });

      assert.ok(typeof validator === 'function');

      const validResult = validator({ data: 'hello' });
      assert.strictEqual(validResult.valid, true);

      const invalidResult = validator({ data: 'hello', extra: 'field' });
      assert.strictEqual(invalidResult.valid, false);
    });

    it('should allow options override', function () {
      const StringClass = rclnodejs.require('std_msgs/msg/String');
      const validator = rclnodejs.createMessageValidator(StringClass, {
        strict: true,
      });

      const result = validator(
        { data: 'hello', extra: 'field' },
        { strict: false }
      );
      assert.strictEqual(result.valid, true);
    });
  });

  describe('MessageValidationError', function () {
    it('should create error with issues', function () {
      const issues = [
        {
          field: 'data',
          problem: 'TYPE_MISMATCH',
          expected: 'string',
          received: 'number',
        },
      ];
      const error = new rclnodejs.MessageValidationError(
        'std_msgs/msg/String',
        issues
      );

      assert.ok(error instanceof rclnodejs.ValidationError);
      assert.ok(error instanceof rclnodejs.MessageValidationError);
      assert.strictEqual(error.messageType, 'std_msgs/msg/String');
      assert.deepStrictEqual(error.issues, issues);
    });

    it('should filter issues by type', function () {
      const issues = [
        {
          field: 'data',
          problem: 'TYPE_MISMATCH',
          expected: 'string',
          received: 'number',
        },
        { field: 'unknown', problem: 'UNKNOWN_FIELD' },
      ];
      const error = new rclnodejs.MessageValidationError(
        'test/msg/Test',
        issues
      );

      const typeIssues = error.getIssuesByType('TYPE_MISMATCH');
      assert.strictEqual(typeIssues.length, 1);
      assert.strictEqual(typeIssues[0].field, 'data');

      const unknownIssues = error.getIssuesByType('UNKNOWN_FIELD');
      assert.strictEqual(unknownIssues.length, 1);
      assert.strictEqual(unknownIssues[0].field, 'unknown');
    });

    it('should check for field issues', function () {
      const issues = [{ field: 'data', problem: 'TYPE_MISMATCH' }];
      const error = new rclnodejs.MessageValidationError(
        'test/msg/Test',
        issues
      );

      assert.strictEqual(error.hasFieldIssue('data'), true);
      assert.strictEqual(error.hasFieldIssue('other'), false);
    });
  });

  describe('Publisher validation integration', function () {
    let node;

    beforeEach(function () {
      node = new rclnodejs.Node('test_validation_node');
    });

    afterEach(function () {
      node.destroy();
    });

    it('should publish without validation by default', function () {
      const publisher = node.createPublisher(
        'std_msgs/msg/String',
        'test_topic'
      );
      assert.strictEqual(publisher.validationEnabled, false);

      assert.doesNotThrow(() => {
        publisher.publish({ data: 'valid string' });
      });
    });

    it('should validate when enabled via options', function () {
      const publisher = node.createPublisher(
        'std_msgs/msg/String',
        'test_topic',
        {
          validateMessages: true,
        }
      );

      assert.strictEqual(publisher.validationEnabled, true);

      assert.doesNotThrow(() => {
        publisher.publish({ data: 'valid string' });
      });

      assert.throws(() => {
        publisher.publish({ data: 123 });
      }, rclnodejs.MessageValidationError);
    });

    it('should validate with per-publish override', function () {
      const publisher = node.createPublisher(
        'std_msgs/msg/String',
        'test_topic'
      );

      assert.throws(() => {
        publisher.publish({ data: 123 }, { validate: true });
      }, rclnodejs.MessageValidationError);

      assert.doesNotThrow(() => {
        publisher.publish({ data: 'valid string' });
      });
    });

    it('should toggle validation with setValidation', function () {
      const publisher = node.createPublisher(
        'std_msgs/msg/String',
        'test_topic'
      );

      publisher.setValidation(true);
      assert.strictEqual(publisher.validationEnabled, true);

      publisher.setValidation(false);
      assert.strictEqual(publisher.validationEnabled, false);
    });
  });
});
