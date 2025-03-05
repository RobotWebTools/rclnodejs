// Copyright (c) 2020 Wayne Parrott. All rights reserved.
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

// Note: parameter api and function based on
// https://design.ros2.org/articles/ros_parameters.html
// https://github.com/ros2/rcl and
// https://github.com/ros2/rclpy

'use strict';

const IsClose = require('is-close');

/**
 * The plus/minus tolerance for determining number equivalence.
 * @constant {number}
 *
 *  @see [FloatingPointRange]{@link FloatingPointRange}
 *  @see [IntegerRange]{@link IntegerRange}
 */
const DEFAULT_NUMERIC_RANGE_TOLERANCE = 1e-6;

const PARAMETER_SEPARATOR = '.';
const PARAMETER_BYTE = 10;

/**
 * Enum for ParameterType
 * @readonly
 * @enum {number}
 */
const ParameterType = {
  /** @member {number} */
  PARAMETER_NOT_SET: 0,
  /** @member {number} */
  PARAMETER_BOOL: 1,
  /** @member {number} */
  PARAMETER_INTEGER: 2,
  /** @member {number} */
  PARAMETER_DOUBLE: 3,
  /** @member {number} */
  PARAMETER_STRING: 4,
  /** @member {number} */
  PARAMETER_BYTE_ARRAY: 5,
  /** @member {number} */
  PARAMETER_BOOL_ARRAY: 6,
  /** @member {number} */
  PARAMETER_INTEGER_ARRAY: 7,
  /** @member {number} */
  PARAMETER_DOUBLE_ARRAY: 8,
  /** @member {number} */
  PARAMETER_STRING_ARRAY: 9,
};

/**
 * A node parameter.
 * @class
 */
class Parameter {
  /**
   * Create a Parameter instance from an rlc_interfaces/msg/Parameter message.
   * @constructs
   * @param {rcl_interfaces/msg/Parameter} parameterMsg - The message to convert to a parameter.
   * @return {Parameter} - The new instance.
   */
  static fromParameterMessage(parameterMsg) {
    const name = parameterMsg.name;
    const type = parameterMsg.value.type;
    let value;

    switch (type) {
      case ParameterType.PARAMETER_NOT_SET:
        break;
      case ParameterType.PARAMETER_BOOL:
        value = parameterMsg.value.bool_value;
        break;
      case ParameterType.PARAMETER_BOOL_ARRAY:
        value = parameterMsg.value.bool_array_value;
        break;
      case ParameterType.PARAMETER_BYTE_ARRAY:
        value = Array.from(parameterMsg.value.byte_array_value);
        break;
      case ParameterType.PARAMETER_DOUBLE:
        value = parameterMsg.value.double_value;
        break;
      case ParameterType.PARAMETER_DOUBLE_ARRAY:
        value = Array.from(parameterMsg.value.double_array_value);
        break;
      case ParameterType.PARAMETER_INTEGER:
        value = parameterMsg.value.integer_value;
        break;
      case ParameterType.PARAMETER_INTEGER_ARRAY:
        value = Array.from(parameterMsg.value.integer_array_value);
        break;
      case ParameterType.PARAMETER_STRING:
        value = parameterMsg.value.string_value;
        break;
      case ParameterType.PARAMETER_STRING_ARRAY:
        value = Array.from(parameterMsg.value.string_array_value);
        break;
    }

    return new Parameter(name, type, value);
  }

  /**
   * Create new parameter instances.
   * @constructor
   *
   * @param {string} name - The parameter name, must be a valid name.
   * @param {ParameterType} type - The type identifier.
   * @param {any} value - The parameter value.
   */
  constructor(name, type, value) {
    this._name = name;
    this._type = type;
    // Convert to bigint if it's type of `PARAMETER_INTEGER`.
    this._value =
      this._type == ParameterType.PARAMETER_INTEGER ? BigInt(value) : value;
    this._isDirty = true;

    this.validate();
  }

  /**
   * Get name
   *
   * @return {string} - The parameter name.
   */
  get name() {
    return this._name;
  }

  /**
   * Get type
   *
   * @return {ParameterType} - The parameter type.
   */
  get type() {
    return this._type;
  }

  /**
   * Get value.
   *
   * @return {any} - The parameter value.
   */
  get value() {
    return this._value;
  }

  /**
   * Set value.
   * Value must be compatible with the type property.
   * @param {any} newValue - The parameter name.
   */
  set value(newValue) {
    // no empty array value allowed
    this._value =
      Array.isArray(newValue) && newValue.length === 0 ? null : newValue;

    this._dirty = true;
    this.validate();
  }

  /**
   * Check the state of this property.
   * Throw TypeError on first property with invalid type.
   * @return {undefined}
   */
  validate() {
    // validate name
    if (
      !this.name ||
      typeof this.name !== 'string' ||
      this.name.trim().length === 0
    ) {
      throw new TypeError('Invalid name');
    }

    // validate type
    if (!validType(this.type)) {
      throw new TypeError('Invalid type');
    }

    // validate value
    if (!validValue(this.value, this.type)) {
      throw new TypeError('Incompatible value.');
    }

    this._dirty = false;
  }

  /**
   * Create a rcl_interfaces.msg.Parameter from this instance.
   *
   * @return {rcl_interfaces.msg.Parameter} - The new instance.
   */
  toParameterMessage() {
    const msg = {
      name: this.name,
      value: this.toParameterValueMessage(),
    };
    return msg;
  }

  /**
   * Create a rcl_interfaces.msg.ParameterValue from this instance.
   *
   * @return {rcl_interfaces.msg.ParameterValue} - The new instance.
   */
  toParameterValueMessage() {
    const msg = {};
    msg.type = this.type;
    switch (this.type) {
      case ParameterType.PARAMETER_NOT_SET:
        break;
      case ParameterType.PARAMETER_BOOL:
        msg.bool_value = this.value;
        break;
      case ParameterType.PARAMETER_BOOL_ARRAY:
        msg.bool_array_value = this.value;
        break;
      case ParameterType.PARAMETER_BYTE_ARRAY:
        msg.byte_array_value = this.value.map((val) => Math.trunc(val));
        break;
      case ParameterType.PARAMETER_DOUBLE:
        msg.double_value = this.value;
        break;
      case ParameterType.PARAMETER_DOUBLE_ARRAY:
        msg.double_array_value = this.value;
        break;
      case ParameterType.PARAMETER_INTEGER:
        msg.integer_value = this.value;
        break;
      case ParameterType.PARAMETER_INTEGER_ARRAY:
        msg.integer_array_value = this.value;
        break;
      case ParameterType.PARAMETER_STRING:
        msg.string_value = this.value;
        break;
      case ParameterType.PARAMETER_STRING_ARRAY:
        msg.string_array_value = this.value;
        break;
    }

    return msg;
  }
}

/**
 * A node parameter descriptor.
 * @class
 */
class ParameterDescriptor {
  /**
   * Create a new instance from a parameter.
   * @constructs
   * @param {Parameter} parameter - The parameter from which new instance is constructed.
   * @return {ParameterDescriptor} - The new instance.
   */
  static fromParameter(parameter) {
    const name = parameter.name;
    const type = parameter.type;
    return new ParameterDescriptor(name, type, 'Created from parameter.');
  }

  /**
   * Create new instances.
   * @constructor
   * @param {string} name - The descriptor name, must be a valid name.
   * @param {ParameterType} type - The type identifier.
   * @param {string} [description] - A descriptive string.
   * @param {boolean} [readOnly] - True indicates a parameter of this type can not be modified. Default = false.
   * @param {Range} [range] - An optional IntegerRange or FloatingPointRange.
   */
  constructor(
    name,
    type = ParameterType.PARAMETER_NOT_SET,
    description = 'no description',
    readOnly = false,
    range = null
  ) {
    this._name = name; // string
    this._type = type; // ParameterType
    this._description = description;
    this._readOnly = readOnly;
    this._additionalConstraints = '';
    this._range = range;

    this.validate();
  }

  /**
   * Get name.
   *
   * @return {string} - The name property.
   */
  get name() {
    return this._name;
  }

  /**
   * Get type.
   *
   * @return {ParameterType} - The type property.
   */
  get type() {
    return this._type;
  }

  /**
   * Get description.
   *
   * @return {string} - A descriptive string property.
   */
  get description() {
    return this._description;
  }

  /**
   * Get readOnly property.
   *
   * @return {boolean} - The readOnly property.
   */
  get readOnly() {
    return this._readOnly;
  }

  /**
   * Get additionalConstraints property.
   *
   * @return {string} - The additionalConstraints property.
   */
  get additionalConstraints() {
    return this._additionalConstraints;
  }

  /**
   * Set additionalConstraints property. .
   *
   * @param {string} constraintDescription - The new value.
   */
  set additionalConstraints(constraintDescription) {
    this._additionalConstraints = constraintDescription;
  }

  /**
   * Determine if rangeConstraint property has been set.
   *
   * @return {boolean} - The rangeConstraint property.
   */
  hasRange() {
    return !!this._range;
  }

  /**
   * Get range.
   *
   * @return {FloatingPointRange|IntegerRange} - The range property.
   */
  get range() {
    return this._range;
  }

  /**
   * Set range.
   * The range must be compatible with the type property.
   * @param {FloatingPointRange|IntegerRange} range - The new range.
   */
  set range(range) {
    if (!range) {
      this._range = null;
      return;
    }
    if (!(range instanceof Range)) {
      throw TypeError('Expected instance of Range.');
    }
    if (!range.isValidType(this.type)) {
      throw TypeError('Incompatible Range');
    }

    this._range = range;
  }

  /**
   * Check the state and ensure it is valid.
   * Throw a TypeError if invalid state is detected.
   *
   * @return {undefined}
   */
  validate() {
    // validate name
    if (
      !this.name ||
      typeof this.name !== 'string' ||
      this.name.trim().length === 0
    ) {
      throw new TypeError('Invalid name');
    }

    // validate type
    if (!validType(this.type)) {
      throw new TypeError('Invalid type');
    }

    // validate description
    if (this.description && typeof this.description !== 'string') {
      throw new TypeError('Invalid description');
    }

    // validate rangeConstraint
    if (this.hasRange() && !this.range.isValidType(this.type)) {
      throw new TypeError('Incompatible Range');
    }
  }

  /**
   * Check a parameter for consistency with this descriptor.
   * Throw an Error if an inconsistent state is detected.
   *
   * @param {Parameter} parameter - The parameter to test for consistency.
   * @return {undefined}
   */
  validateParameter(parameter) {
    if (!parameter) {
      throw new TypeError('Parameter is undefined');
    }

    // ensure parameter is valid
    try {
      parameter.validate();
    } catch {
      throw new TypeError('Parameter is invalid');
    }

    // ensure this descriptor is valid
    try {
      this.validate();
    } catch {
      throw new Error('Descriptor is invalid.');
    }

    if (this.name !== parameter.name) throw new Error('Name mismatch');
    if (this.type !== parameter.type) throw new Error('Type mismatch');
    if (this.hasRange() && !this.range.inRange(parameter.value)) {
      throw new RangeError('Parameter value is not in descriptor range');
    }
  }

  /**
   * Create a rcl_interfaces.msg.ParameterDescriptor from this descriptor.
   *
   * @return {rcl_interfaces.msg.ParameterDescriptor} - The new message.
   */
  toMessage() {
    const msg = {
      name: this.name,
      type: this.type,
      description: this.description,
      additional_constraints: this.additionalConstraints,
      read_only: this.readOnly,
    };
    if (
      (this._type === ParameterType.PARAMETER_INTEGER ||
        this._type === ParameterType.PARAMETER_INTEGER_ARRAY) &&
      this._rangeConstraint instanceof IntegerRange
    ) {
      msg.integer_range = [this._rangeConstraint];
    } else if (
      (this._type === ParameterType.PARAMETER_DOUBLE ||
        this._type === ParameterType.PARAMETER_DOUBLE_ARRAY) &&
      this._rangeConstraint instanceof FloatingPointRange
    ) {
      msg.floating_point_range = [this._rangeConstraint];
    }

    return msg;
  }
}

/**
 * An abstract class defining a range of numbers between 2 points inclusively
 * divided by a step value.
 * @class
 */
class Range {
  /**
   * Create a new instance.
   * @constructor
   * @param {number} fromValue - The lowest inclusive value in range
   * @param {number} toValue - The highest inclusive value in range
   * @param {number} step - The internal unit size.
   */
  constructor(fromValue, toValue, step = 1) {
    this._fromValue = fromValue;
    this._toValue = toValue;
    this._step = step;
  }

  /**
   * Get fromValue.
   *
   * @return {number} - The lowest inclusive value in range.
   */
  get fromValue() {
    return this._fromValue;
  }

  /**
   * Get toValue.
   *
   * @return {number} - The highest inclusive value in range.
   */
  get toValue() {
    return this._toValue;
  }

  /**
   * Get step unit.
   *
   * @return {number} - The internal unit size.
   */
  get step() {
    return this._step;
  }

  /**
   * Determine if a value is within this range.
   * A TypeError is thrown when value is not a number or bigint.
   * Subclasses should override and call this method for basic type checking.
   *
   * @param {number|bigint} value - The number or bigint to check.
   * @return {boolean} - True if value satisfies the range; false otherwise.
   */
  inRange(value) {
    if (Array.isArray(value)) {
      const valArr = value;
      return valArr.reduce(
        (inRange, val) => inRange && this.inRange(val),
        true
      );
    } else if (typeof value !== 'number' && typeof value !== 'bigint') {
      throw new TypeError('Value must be a number or bigint');
    }

    return true;
  }

  /**
   * Abstract method that determines if a ParameterType is compatible.
   * Subclasses must implement this method.
   *
   * @param {ParameterType} parameterType - The parameter type to test.
   * @return {boolean} - True if parameterType is compatible; otherwise return false.
   */
  // eslint-disable-next-line no-unused-vars
  isValidType(parameterType) {
    return false;
  }
}

/**
 * Defines a range for floating point values.
 * @class
 */
class FloatingPointRange extends Range {
  /**
   * Create a new instance.
   * @constructor
   * @param {number} fromValue - The lowest inclusive value in range
   * @param {number} toValue - The highest inclusive value in range
   * @param {number} step - The internal unit size.
   * @param {number} tolerance - The plus/minus tolerance for number equivalence.
   */
  constructor(
    fromValue,
    toValue,
    step = 1,
    tolerance = DEFAULT_NUMERIC_RANGE_TOLERANCE
  ) {
    super(fromValue, toValue, step);
    this._tolerance = tolerance;
  }

  get tolerance() {
    return this._tolerance;
  }

  /**
   * Determine if a ParameterType is compatible.
   *
   * @param {ParameterType} parameterType - The parameter type to test.
   * @return {boolean} - True if parameterType is compatible; otherwise return false.
   */
  isValidType(parameterType) {
    const result =
      parameterType === ParameterType.PARAMETER_DOUBLE ||
      parameterType === ParameterType.PARAMETER_DOUBLE_ARRAY;
    return result;
  }

  /**
   * Determine if a value is within this range.
   * A TypeError is thrown when value is not a number.
   *
   * @param {number} value - The number to check.
   * @return {boolean} - True if value satisfies the range; false otherwise.
   */
  inRange(value) {
    if (!super.inRange(value)) return false;

    const min = Math.min(this.fromValue, this.toValue);
    const max = Math.max(this.fromValue, this.toValue);

    if (
      IsClose.isClose(value, min, this.tolerance) ||
      IsClose.isClose(value, max, this.tolerance)
    ) {
      return true;
    }
    if (value < min || value > max) {
      return false;
    }
    if (this.step != 0.0) {
      const distanceInSteps = Math.round((value - min) / this.step);
      if (
        !IsClose.isClose(
          min + distanceInSteps * this.step,
          value,
          this.tolerance
        )
      ) {
        return false;
      }
    }

    return true;
  }
}

/**
 * Defines a range for integer values.
 * @class
 */
class IntegerRange extends Range {
  /**
   * Create a new instance.
   * @constructor
   * @param {bigint} fromValue - The lowest inclusive value in range
   * @param {bigint} toValue - The highest inclusive value in range
   * @param {bigint} step - The internal unit size.
   */
  constructor(fromValue, toValue, step = 1n) {
    super(fromValue, toValue, step);
  }

  /**
   * Determine if a ParameterType is compatible.
   *
   * @param {ParameterType} parameterType - The parameter type to test.
   * @return {boolean} - True if parameterType is compatible; otherwise return false.
   */
  isValidType(parameterType) {
    const result =
      parameterType === ParameterType.PARAMETER_INTEGER ||
      parameterType === ParameterType.PARAMETER_INTEGER_ARRAY;
    return result;
  }

  inRange(value) {
    const min = this.fromValue;
    const max = this.toValue;
    if (value < min || value > max) {
      return false;
    }

    if (this.step != 0n && (value - min) % this.step !== 0n) {
      return false;
    }
    return true;
  }
}

/**
 * Infer a ParameterType for JS primitive types:
 * string, boolean, number and arrays of these types.
 * A TypeError is thrown for a value who's type is not one of
 * the set listed.
 * @param {any} value - The value to infer it's ParameterType
 * @returns {ParameterType} - The ParameterType that best scribes the value.
 */
// eslint-disable-next-line no-unused-vars
function parameterTypeFromValue(value) {
  if (!value) return ParameterType.PARAMETER_NOT_SET;
  if (typeof value === 'boolean') return ParameterType.PARAMETER_BOOL;
  if (typeof value === 'string') return ParameterType.PARAMETER_STRING;
  if (typeof value === 'number') return ParameterType.PARAMETER_DOUBLE;
  if (Array.isArray(value)) {
    if (value.length > 0) {
      const elementType = parameterTypeFromValue(value[0]);
      switch (elementType) {
        case ParameterType.PARAMETER_BOOL:
          return ParameterType.PARAMETER_BOOL_ARRAY;
        case ParameterType.PARAMETER_DOUBLE:
          return ParameterType.PARAMETER_DOUBLE_ARRAY;
        case ParameterType.PARAMETER_STRING:
          return ParameterType.PARAMETER_STRING_ARRAY;
      }
    }

    return ParameterType.PARAMETER_NOT_SET;
  }

  // otherwise unrecognized value
  throw new TypeError('Unrecognized parameter type.');
}

/**
 * Determine if a number maps to is a valid ParameterType.
 *
 * @param {ParameterType} parameterType - The value to test.
 * @return {boolean} - True if value is a valid ParameterType; false otherwise.
 */
function validType(parameterType) {
  let result =
    typeof parameterType === 'number' &&
    ParameterType.PARAMETER_NOT_SET <=
      parameterType <=
      ParameterType.PARAMETER_STRING_ARRAY;

  return result;
}

/**
 * Test if value can be represented by a ParameterType.
 *
 * @param {number} value - The value to test.
 * @param {ParameterType} type - The ParameterType to test value against.
 * @return {boolean} - True if value can be represented by type.
 */
function validValue(value, type) {
  if (value == null) {
    return type === ParameterType.PARAMETER_NOT_SET;
  }

  let result = true;
  switch (type) {
    case ParameterType.PARAMETER_NOT_SET:
      result = !value;
      break;
    case ParameterType.PARAMETER_BOOL:
      result = typeof value === 'boolean';
      break;
    case ParameterType.PARAMETER_STRING:
      result = typeof value === 'string';
      break;
    case ParameterType.PARAMETER_DOUBLE:
    case PARAMETER_BYTE:
      result = typeof value === 'number';
      break;
    case ParameterType.PARAMETER_INTEGER:
      result = typeof value === 'bigint';
      break;
    case ParameterType.PARAMETER_BOOL_ARRAY:
    case ParameterType.PARAMETER_BYTE_ARRAY:
    case ParameterType.PARAMETER_INTEGER_ARRAY:
    case ParameterType.PARAMETER_DOUBLE_ARRAY:
    case ParameterType.PARAMETER_STRING_ARRAY:
      result = _validArray(value, type);
      break;
    default:
      result = false;
  }

  return result;
}

function _validArray(values, type) {
  if (!Array.isArray(values)) return false;

  let arrayElementType;
  if (type === ParameterType.PARAMETER_BOOL_ARRAY) {
    arrayElementType = ParameterType.PARAMETER_BOOL;
  } else if (type === ParameterType.PARAMETER_BYTE_ARRAY) {
    arrayElementType = PARAMETER_BYTE;
  }
  if (type === ParameterType.PARAMETER_INTEGER_ARRAY) {
    arrayElementType = ParameterType.PARAMETER_INTEGER;
  }
  if (type === ParameterType.PARAMETER_DOUBLE_ARRAY) {
    arrayElementType = ParameterType.PARAMETER_DOUBLE;
  }
  if (type === ParameterType.PARAMETER_STRING_ARRAY) {
    arrayElementType = ParameterType.PARAMETER_STRING;
  }

  return values.reduce(
    (compatible, val) =>
      compatible &&
      !_isArrayParameterType(arrayElementType) &&
      validValue(val, arrayElementType),
    true
  );
}

function _isArrayParameterType(type) {
  return (
    type === ParameterType.PARAMETER_BOOL_ARRAY ||
    type === ParameterType.PARAMETER_BYTE_ARRAY ||
    type === ParameterType.PARAMETER_INTEGER_ARRAY ||
    type === ParameterType.PARAMETER_DOUBLE_ARRAY ||
    type === ParameterType.PARAMETER_STRING_ARRAY
  );
}

module.exports = {
  ParameterType,
  Parameter,
  ParameterDescriptor,
  PARAMETER_SEPARATOR,
  Range,
  FloatingPointRange,
  IntegerRange,
  DEFAULT_NUMERIC_RANGE_TOLERANCE,
};
