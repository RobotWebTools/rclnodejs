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

describe('MessageValidation unit testing', function () {
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
  });
});
