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
const {
  MessageValidationError,
  TypeValidationError,
} = require('../lib/errors.js');
const {
  assertValidMessage,
  validateMessage,
  createMessageValidator,
  getFieldNames,
  getMessageSchema,
  ValidationProblem,
  getMessageTypeString,
} = require('../lib/message_validation.js');

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
    let testCounter = 0;

    beforeEach(function () {
      testCounter++;
      node = new rclnodejs.Node(
        `test_validation_node_${testCounter}_${Date.now()}`
      );
    });

    afterEach(function () {
      node.destroy();
    });

    it('should publish without validation by default', function () {
      const publisher = node.createPublisher(
        'std_msgs/msg/String',
        'test_topic'
      );
      assert.strictEqual(publisher.willValidateMessage, false);

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

      assert.strictEqual(publisher.willValidateMessage, true);

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

    it('should toggle validation with willValidateMessage', function () {
      const publisher = node.createPublisher(
        'std_msgs/msg/String',
        'test_topic'
      );

      publisher.willValidateMessage = true;
      assert.strictEqual(publisher.willValidateMessage, true);

      publisher.willValidateMessage = false;
      assert.strictEqual(publisher.willValidateMessage, false);
    });
  });
});

describe('MessageValidation unit testing (Mocks)', function () {
  const MockStringMsg = class {
    static get ROSMessageDef() {
      return {
        fields: [
          { name: 'data', type: { type: 'string', isPrimitiveType: true } },
        ],
        constants: [],
        baseType: { pkgName: 'std_msgs', type: 'String' },
      };
    }
    static type() {
      return { pkgName: 'std_msgs', subFolder: 'msg', interfaceName: 'String' };
    }
  };

  const MockArrayMsg = class {
    static get ROSMessageDef() {
      return {
        fields: [
          {
            name: 'covariance',
            type: {
              type: 'float64',
              isPrimitiveType: true,
              isArray: true,
              isFixedSizeArray: true,
              arraySize: 3,
            },
          },
          {
            name: 'unbounded',
            type: {
              type: 'int32',
              isPrimitiveType: true,
              isArray: true,
            },
          },
        ],
        constants: [],
      };
    }
    static type() {
      return {
        pkgName: 'test_pkg',
        subFolder: 'msg',
        interfaceName: 'ArrayTest',
      };
    }
  };

  const MockBoundArrayMsg = class {
    static get ROSMessageDef() {
      return {
        fields: [
          {
            name: 'values',
            type: {
              type: 'int32',
              isPrimitiveType: true,
              isArray: true,
              isUpperBound: true,
              arraySize: 5,
            },
          },
        ],
        constants: [],
      };
    }
    static type() {
      return {
        pkgName: 'test_pkg',
        subFolder: 'msg',
        interfaceName: 'BoundArrayTest',
      };
    }
  };

  const MockInt64Msg = class {
    static get ROSMessageDef() {
      return {
        fields: [
          { name: 'id', type: { type: 'int64', isPrimitiveType: true } },
        ],
        constants: [],
      };
    }
    static type() {
      return {
        pkgName: 'test_pkg',
        subFolder: 'msg',
        interfaceName: 'Int64Test',
      };
    }
  };

  const MockNestedArrayMsg = class {
    constructor() {
      this.elements = [];
      // Mock the property used by getNestedTypeClass for arrays
      this.elements.classType = { elementType: MockStringMsg };
    }
    static get ROSMessageDef() {
      return {
        fields: [
          {
            name: 'elements',
            type: {
              type: 'String',
              pkgName: 'std_msgs',
              isPrimitiveType: false,
              isArray: true,
            },
          },
        ],
        constants: [],
      };
    }
    static type() {
      return {
        pkgName: 'test_pkg',
        subFolder: 'msg',
        interfaceName: 'NestedArrayTest',
      };
    }
  };

  describe('Utility functions', function () {
    it('getMessageTypeString returns correct string', function () {
      const str = getMessageTypeString(MockStringMsg);
      assert.strictEqual(str, 'std_msgs/msg/String');
    });

    it('getMessageTypeString returns unknown for invalid input', function () {
      const str = getMessageTypeString({});
      assert.strictEqual(str, 'unknown');
    });

    it('getMessageSchema returns schema', function () {
      const schema = getMessageSchema(MockStringMsg);
      assert.strictEqual(schema.messageType, 'std_msgs/msg/String');
      assert.ok(schema.fields.length > 0);
    });

    it('getFieldNames returns field names', function () {
      const names = getFieldNames(MockStringMsg);
      assert.deepStrictEqual(names, ['data']);
    });
  });

  describe('createMessageValidator', function () {
    it('creates a function', function () {
      const validator = createMessageValidator(MockStringMsg);
      assert.strictEqual(typeof validator, 'function');
    });

    it('throws on invalid type class', function () {
      assert.throws(() => {
        createMessageValidator(null);
      }, TypeValidationError);
    });

    it('validator validates correctly', function () {
      const validator = createMessageValidator(MockStringMsg);
      const result = validator({ data: 'ok' });
      assert.strictEqual(result.valid, true);

      const resultFail = validator({ data: 123 });
      assert.strictEqual(resultFail.valid, false);
    });
  });

  describe('assertValidMessage', function () {
    it('throws Validation Error when validation fails', function () {
      const msg = { data: 123 };

      assert.throws(() => {
        assertValidMessage(msg, MockStringMsg);
      }, MessageValidationError);
    });

    it('passes for valid message', function () {
      const msg = { data: 'hello' };

      assert.doesNotThrow(() => {
        assertValidMessage(msg, MockStringMsg);
      });
    });
  });

  describe('validateMessage', function () {
    it('detects unknown fields (strict mode)', function () {
      const plainMsg = { data: 'hello', unknown: 1 };

      const result = validateMessage(plainMsg, MockStringMsg, { strict: true });
      assert.strictEqual(result.valid, false);
      assert.strictEqual(
        result.issues[0].problem,
        ValidationProblem.UNKNOWN_FIELD
      );
    });

    it('detects type mismatch', function () {
      const plainMsg = { data: 123 };
      const result = validateMessage(plainMsg, MockStringMsg, {
        checkTypes: true,
      });
      assert.strictEqual(result.valid, false);
      assert.strictEqual(
        result.issues[0].problem,
        ValidationProblem.TYPE_MISMATCH
      );
    });

    it('validates primitive wrapper check (single field optimization)', function () {
      // Test single primitive value directly
      const result = validateMessage('mystring', MockStringMsg);
      assert.strictEqual(result.valid, true);

      const resultFail = validateMessage(123, MockStringMsg);
      assert.strictEqual(resultFail.valid, false);
    });

    it('validates array constraints (fixed size)', function () {
      const plainMsg = { covariance: [1, 2] }; // Too short (needs 3)
      // Note: We need to omit 'unbounded' or provide it validly

      const result = validateMessage(plainMsg, MockArrayMsg, {
        checkTypes: true,
      });
      const issue = result.issues.find(
        (i) => i.problem === ValidationProblem.ARRAY_LENGTH
      );
      assert.ok(issue);
    });

    it('validates array constraints (type mismatch in array)', function () {
      const plainMsg = { covariance: [1, 2, 'bad'] };
      const result = validateMessage(plainMsg, MockArrayMsg, {
        checkTypes: true,
      });
      const issue = result.issues.find(
        (i) => i.problem === ValidationProblem.TYPE_MISMATCH
      );
      assert.ok(issue);
    });

    it('detects missing required fields', function () {
      const plainMsg = {};
      const result = validateMessage(plainMsg, MockStringMsg, {
        checkRequired: true,
      });
      assert.strictEqual(result.valid, false);
      assert.strictEqual(
        result.issues[0].problem,
        ValidationProblem.MISSING_FIELD
      );
    });

    it('handles null/undefined object', function () {
      const result = validateMessage(null, MockStringMsg);
      assert.strictEqual(result.valid, false);
    });

    it('validates array constraints (upper bound)', function () {
      const validMsg = { values: [1, 2, 3, 4, 5] };
      const validResult = validateMessage(validMsg, MockBoundArrayMsg);
      assert.strictEqual(validResult.valid, true);

      const invalidMsg = { values: [1, 2, 3, 4, 5, 6] };
      const invalidResult = validateMessage(invalidMsg, MockBoundArrayMsg);
      assert.strictEqual(invalidResult.valid, false);
      assert.strictEqual(
        invalidResult.issues[0].problem,
        ValidationProblem.ARRAY_LENGTH
      );
    });

    it('allows TypedArray for array fields', function () {
      const msg = { values: new Int32Array([1, 2, 3]) };
      const result = validateMessage(msg, MockBoundArrayMsg);
      assert.strictEqual(result.valid, true);
    });

    it('allows number for int64/uint64 (BigInt) fields', function () {
      const msg = { id: 12345 }; // JS Number
      const result = validateMessage(msg, MockInt64Msg, { checkTypes: true });
      assert.strictEqual(result.valid, true);
    });

    it('detects type mismatch for int64', function () {
      const msg = { id: 'not-a-number' };
      const result = validateMessage(msg, MockInt64Msg, { checkTypes: true });
      assert.strictEqual(result.valid, false);
      assert.strictEqual(
        result.issues[0].problem,
        ValidationProblem.TYPE_MISMATCH
      );
    });

    it('validates nested message arrays', function () {
      const validMsg = {
        elements: [{ data: 'str1' }, { data: 'str2' }],
      };
      const validResult = validateMessage(validMsg, MockNestedArrayMsg);
      assert.strictEqual(validResult.valid, true);
    });

    it('detects errors in nested message arrays', function () {
      const invalidMsg = {
        elements: [{ data: 'str1' }, { data: 123 }], // 2nd element invalid
      };
      const result = validateMessage(invalidMsg, MockNestedArrayMsg, {
        checkTypes: true,
      });

      assert.strictEqual(result.valid, false);
      const issue = result.issues[0];
      assert.ok(issue.field.includes('elements[1]'));
      assert.strictEqual(issue.problem, ValidationProblem.TYPE_MISMATCH);
    });

    it('validates array of primitives and detects element type error', function () {
      // Re-use MockBoundArrayMsg (int32 array)
      const invalidMsg = { values: [1, 'bad', 3] };
      const result = validateMessage(invalidMsg, MockBoundArrayMsg, {
        checkTypes: true,
      });
      assert.strictEqual(result.valid, false);
      const issue = result.issues[0];
      assert.ok(issue.field.includes('values[1]'));
      assert.strictEqual(issue.problem, ValidationProblem.TYPE_MISMATCH);
    });

    it('reports error when schema is missing', function () {
      const NoSchemaClass = class {};
      const result = validateMessage({}, NoSchemaClass);
      assert.strictEqual(result.valid, false);
      assert.strictEqual(result.issues[0].problem, 'NO_SCHEMA');
    });

    it('reports error when type class is invalid', function () {
      const result = validateMessage({}, 'ValidLookingStringButNotLoaded');
      // resolveTypeClass tends to return null if loadInterface fails for string
      assert.strictEqual(result.valid, false);
      assert.strictEqual(result.issues[0].problem, 'INVALID_TYPE_CLASS');
    });
  });
});
