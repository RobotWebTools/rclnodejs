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

const { MessageValidationError, TypeValidationError } = require('./errors.js');
const interfaceLoader = require('./interface_loader.js');

/**
 * Validation issue problem types
 * @enum {string}
 */
const ValidationProblem = {
  /** Field exists in object but not in message schema */
  UNKNOWN_FIELD: 'UNKNOWN_FIELD',
  /** Field type doesn't match expected type */
  TYPE_MISMATCH: 'TYPE_MISMATCH',
  /** Required field is missing */
  MISSING_FIELD: 'MISSING_FIELD',
  /** Array length constraint violated */
  ARRAY_LENGTH: 'ARRAY_LENGTH',
  /** Value is out of valid range */
  OUT_OF_RANGE: 'OUT_OF_RANGE',
  /** Nested message validation failed */
  NESTED_ERROR: 'NESTED_ERROR',
};

/**
 * Map ROS primitive types to JavaScript types
 */
const PRIMITIVE_TYPE_MAP = {
  bool: 'boolean',
  int8: 'number',
  uint8: 'number',
  int16: 'number',
  uint16: 'number',
  int32: 'number',
  uint32: 'number',
  int64: 'bigint',
  uint64: 'bigint',
  float32: 'number',
  float64: 'number',
  char: 'number',
  byte: 'number',
  string: 'string',
  wstring: 'string',
};

/**
 * Check if value is a TypedArray
 * @param {any} value - Value to check
 * @returns {boolean} True if TypedArray
 */
function isTypedArray(value) {
  return ArrayBuffer.isView(value) && !(value instanceof DataView);
}

/**
 * Get the JavaScript type string for a value
 * @param {any} value - Value to get type of
 * @returns {string} Type description
 */
function getValueType(value) {
  if (value === null) return 'null';
  if (value === undefined) return 'undefined';
  if (Array.isArray(value)) return 'array';
  if (isTypedArray(value)) return 'TypedArray';
  return typeof value;
}

/**
 * Resolve a type class from various input formats
 * @param {string|object|function} typeClass - Type identifier
 * @returns {function|null} The resolved type class or null
 */
function resolveTypeClass(typeClass) {
  if (typeof typeClass === 'function') {
    return typeClass;
  }

  try {
    return interfaceLoader.loadInterface(typeClass);
  } catch {
    return null;
  }
}

/**
 * Get message type string from type class
 * @param {function} typeClass - Message type class
 * @returns {string} Message type string (e.g., 'std_msgs/msg/String')
 */
function getMessageTypeString(typeClass) {
  if (typeof typeClass.type === 'function') {
    const t = typeClass.type();
    return `${t.pkgName}/${t.subFolder}/${t.interfaceName}`;
  }
  return 'unknown';
}

/**
 * Get the schema definition for a message type
 * @param {function|string|object} typeClass - Message type class or identifier
 * @returns {object|null} Schema definition with fields and constants, or null if not found
 * @example
 * const schema = getMessageSchema(StringClass);
 * // Returns: {
 * //   fields: [{name: 'data', type: {type: 'string', isPrimitiveType: true, ...}}],
 * //   constants: [],
 * //   messageType: 'std_msgs/msg/String'
 * // }
 */
function getMessageSchema(typeClass) {
  const resolved = resolveTypeClass(typeClass);
  if (!resolved || !resolved.ROSMessageDef) {
    return null;
  }

  const def = resolved.ROSMessageDef;
  return {
    fields: def.fields || [],
    constants: def.constants || [],
    messageType: getMessageTypeString(resolved),
    baseType: def.baseType,
  };
}

/**
 * Get field names for a message type
 * @param {function|string|object} typeClass - Message type class or identifier
 * @returns {string[]} Array of field names
 */
function getFieldNames(typeClass) {
  const schema = getMessageSchema(typeClass);
  if (!schema) return [];
  return schema.fields.map((f) => f.name);
}

/**
 * Get type information for a specific field
 * @param {function|string|object} typeClass - Message type class or identifier
 * @param {string} fieldName - Name of the field
 * @returns {object|null} Field type information or null if not found
 */
function getFieldType(typeClass, fieldName) {
  const schema = getMessageSchema(typeClass);
  if (!schema) return null;

  const field = schema.fields.find((f) => f.name === fieldName);
  return field ? field.type : null;
}

/**
 * Validate a primitive value against its expected type
 * @param {any} value - Value to validate
 * @param {object} fieldType - Field type definition
 * @returns {object|null} Validation issue or null if valid
 */
function validatePrimitiveValue(value, fieldType) {
  const expectedJsType = PRIMITIVE_TYPE_MAP[fieldType.type];
  const actualType = typeof value;

  if (!expectedJsType) {
    return null; // Unknown primitive type, skip validation
  }

  // Allow number for bigint fields (will be converted)
  if (expectedJsType === 'bigint' && actualType === 'number') {
    return null;
  }

  if (actualType !== expectedJsType) {
    return {
      problem: ValidationProblem.TYPE_MISMATCH,
      expected: expectedJsType,
      received: actualType,
    };
  }

  return null;
}

/**
 * Validate array constraints
 * @param {any} value - Array value to validate
 * @param {object} fieldType - Field type definition
 * @returns {object|null} Validation issue or null if valid
 */
function validateArrayConstraints(value, fieldType) {
  if (!Array.isArray(value) && !isTypedArray(value)) {
    return {
      problem: ValidationProblem.TYPE_MISMATCH,
      expected: 'array',
      received: getValueType(value),
    };
  }

  const length = value.length;

  // Fixed size array
  if (fieldType.isFixedSizeArray && length !== fieldType.arraySize) {
    return {
      problem: ValidationProblem.ARRAY_LENGTH,
      expected: `exactly ${fieldType.arraySize} elements`,
      received: `${length} elements`,
    };
  }

  // Upper bound array
  if (fieldType.isUpperBound && length > fieldType.arraySize) {
    return {
      problem: ValidationProblem.ARRAY_LENGTH,
      expected: `at most ${fieldType.arraySize} elements`,
      received: `${length} elements`,
    };
  }

  return null;
}

/**
 * Validate a message object against its schema
 * @param {object} obj - Plain object to validate
 * @param {function|string|object} typeClass - Message type class or identifier
 * @param {object} [options] - Validation options
 * @param {boolean} [options.strict=false] - If true, unknown fields cause validation failure
 * @param {boolean} [options.checkTypes=true] - If true, validate field types
 * @param {boolean} [options.checkRequired=false] - If true, check for missing fields
 * @param {string} [options.path=''] - Current path for nested validation (internal use)
 * @returns {{valid: boolean, issues: Array<object>}} Validation result
 */
function validateMessage(obj, typeClass, options = {}) {
  const {
    strict = false,
    checkTypes = true,
    checkRequired = false,
    path = '',
  } = options;

  const issues = [];
  const resolved = resolveTypeClass(typeClass);

  if (!resolved) {
    issues.push({
      field: path || '(root)',
      problem: 'INVALID_TYPE_CLASS',
      expected: 'valid message type class',
      received: typeof typeClass,
    });
    return { valid: false, issues };
  }

  const schema = getMessageSchema(resolved);
  if (!schema) {
    issues.push({
      field: path || '(root)',
      problem: 'NO_SCHEMA',
      expected: 'message with ROSMessageDef',
      received: 'class without schema',
    });
    return { valid: false, issues };
  }

  if (obj === null || obj === undefined) {
    issues.push({
      field: path || '(root)',
      problem: ValidationProblem.TYPE_MISMATCH,
      expected: 'object',
      received: String(obj),
    });
    return { valid: false, issues };
  }

  const type = typeof obj;
  if (
    type === 'string' ||
    type === 'number' ||
    type === 'boolean' ||
    type === 'bigint'
  ) {
    if (schema.fields.length === 1 && schema.fields[0].name === 'data') {
      const fieldType = schema.fields[0].type;
      if (checkTypes && fieldType.isPrimitiveType) {
        const typeIssue = validatePrimitiveValue(obj, fieldType);
        if (typeIssue) {
          issues.push({
            field: path ? `${path}.data` : 'data',
            ...typeIssue,
          });
        }
      }
      return { valid: issues.length === 0, issues };
    }
  }

  if (type !== 'object') {
    issues.push({
      field: path || '(root)',
      problem: ValidationProblem.TYPE_MISMATCH,
      expected: 'object',
      received: type,
    });
    return { valid: false, issues };
  }

  const fieldNames = new Set(schema.fields.map((f) => f.name));
  const objKeys = Object.keys(obj);

  if (strict) {
    for (const key of objKeys) {
      if (!fieldNames.has(key)) {
        issues.push({
          field: path ? `${path}.${key}` : key,
          problem: ValidationProblem.UNKNOWN_FIELD,
        });
      }
    }
  }

  for (const field of schema.fields) {
    const fieldPath = path ? `${path}.${field.name}` : field.name;
    const value = obj[field.name];
    const fieldType = field.type;

    if (field.name.startsWith('_')) continue;

    if (value === undefined) {
      if (checkRequired) {
        issues.push({
          field: fieldPath,
          problem: ValidationProblem.MISSING_FIELD,
          expected: fieldType.type,
        });
      }
      continue;
    }

    if (fieldType.isArray) {
      const arrayIssue = validateArrayConstraints(value, fieldType);
      if (arrayIssue) {
        issues.push({ field: fieldPath, ...arrayIssue });
        continue;
      }

      if (checkTypes && Array.isArray(value) && value.length > 0) {
        if (fieldType.isPrimitiveType) {
          for (let i = 0; i < value.length; i++) {
            const elemIssue = validatePrimitiveValue(value[i], fieldType);
            if (elemIssue) {
              issues.push({
                field: `${fieldPath}[${i}]`,
                ...elemIssue,
              });
            }
          }
        } else {
          for (let i = 0; i < value.length; i++) {
            const nestedResult = validateMessage(
              value[i],
              getNestedTypeClass(resolved, field.name),
              {
                strict,
                checkTypes,
                checkRequired,
                path: `${fieldPath}[${i}]`,
              }
            );
            if (!nestedResult.valid) {
              issues.push(...nestedResult.issues);
            }
          }
        }
      }
    } else if (fieldType.isPrimitiveType) {
      if (checkTypes) {
        const typeIssue = validatePrimitiveValue(value, fieldType);
        if (typeIssue) {
          issues.push({ field: fieldPath, ...typeIssue });
        }
      }
    } else {
      if (value !== null && typeof value === 'object') {
        const nestedTypeClass = getNestedTypeClass(resolved, field.name);
        if (nestedTypeClass) {
          const nestedResult = validateMessage(value, nestedTypeClass, {
            strict,
            checkTypes,
            checkRequired,
            path: fieldPath,
          });
          if (!nestedResult.valid) {
            issues.push(...nestedResult.issues);
          }
        }
      } else if (checkTypes && value !== null) {
        issues.push({
          field: fieldPath,
          problem: ValidationProblem.TYPE_MISMATCH,
          expected: 'object',
          received: getValueType(value),
        });
      }
    }
  }

  return { valid: issues.length === 0, issues };
}

/**
 * Get the type class for a nested field
 * @param {function} parentTypeClass - Parent message type class
 * @param {string} fieldName - Field name
 * @returns {function|null} Nested type class or null
 */
function getNestedTypeClass(parentTypeClass, fieldName) {
  try {
    const instance = new parentTypeClass();
    const fieldValue = instance[fieldName];

    if (
      fieldValue &&
      fieldValue.constructor &&
      fieldValue.constructor.ROSMessageDef
    ) {
      return fieldValue.constructor;
    }

    if (
      fieldValue &&
      fieldValue.classType &&
      fieldValue.classType.elementType
    ) {
      return fieldValue.classType.elementType;
    }
  } catch {
    const schema = getMessageSchema(parentTypeClass);
    if (schema) {
      const field = schema.fields.find((f) => f.name === fieldName);
      if (field && !field.type.isPrimitiveType) {
        const typeName = `${field.type.pkgName}/msg/${field.type.type}`;
        return resolveTypeClass(typeName);
      }
    }
  }
  return null;
}

/**
 * Validate a message and throw if invalid
 * @param {object} obj - Plain object to validate
 * @param {function|string|object} typeClass - Message type class or identifier
 * @param {object} [options] - Validation options (same as validateMessage)
 * @throws {MessageValidationError} If validation fails
 * @returns {void}
 */
function assertValidMessage(obj, typeClass, options = {}) {
  const result = validateMessage(obj, typeClass, options);

  if (!result.valid) {
    const resolved = resolveTypeClass(typeClass);
    const messageType = resolved
      ? getMessageTypeString(resolved)
      : String(typeClass);
    throw new MessageValidationError(messageType, result.issues);
  }
}

/**
 * Create a validator function for a specific message type
 * @param {function|string|object} typeClass - Message type class or identifier
 * @param {object} [defaultOptions] - Default validation options
 * @returns {function} Validator function that takes (obj, options?) and returns validation result
 */
function createMessageValidator(typeClass, defaultOptions = {}) {
  const resolved = resolveTypeClass(typeClass);
  if (!resolved) {
    throw new TypeValidationError(
      'typeClass',
      typeClass,
      'valid message type class'
    );
  }

  return function validator(obj, options = {}) {
    return validateMessage(obj, resolved, {
      ...defaultOptions,
      ...options,
    });
  };
}

module.exports = {
  ValidationProblem,
  getMessageSchema,
  getFieldNames,
  getFieldType,
  validateMessage,
  assertValidMessage,
  createMessageValidator,
  getMessageTypeString,
};
