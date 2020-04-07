/* eslint-disable camelcase */

declare module 'rclnodejs' {
  /** 
   * The plus/minus tolerance for determining number equivalence.

   *  @remarks
   *  See {@FloatingPointRange | FloatingPointRange}
   *  See {@IntegerRange | IntegerRange}
   */
  export const DEFAULT_NUMERIC_RANGE_TOLERANCE = 1e-6;

  /**
   * Type identifier for a parameter.
   */
  export const enum ParameterType {
    PARAMETER_NOT_SET = 0,
    PARAMETER_BOOL = 1,
    PARAMETER_INTEGER = 2,
    PARAMETER_DOUBLE = 3,
    PARAMETER_STRING = 4,
    PARAMETER_BYTE_ARRAY = 5,
    PARAMETER_BOOL_ARRAY = 6,
    PARAMETER_INTEGER_ARRAY = 7,
    PARAMETER_DOUBLE_ARRAY = 8,
    PARAMETER_STRING_ARRAY = 9,
  }

  /**
   * A node parameter.
   */
  class Parameter {
    /**
     * Create a Parameter instance from an rlc_interfaces/msg/Parameter message.
     *
     * @param parameterMsg - The message to convert to a parameter.
     * @returns The new instance.
     */
    static fromParameterMessage(
      parameterMsg: rcl_interfaces.msg.Parameter
    ): Parameter;

    /**
     * Create new parameter instances.
     *
     * @param name - The parameter name, must be a valid name.
     * @param type - The type identifier.
     * @param {value - The parameter value.
     */
    constructor(name: string, type: ParameterType, value?: any);

    /**
     * The parameter name.
     */
    readonly name: string;

    /**
     *  The parameter type.
     */
    readonly type: ParameterType;

    /**
     * The parameter value.
     * Value must be compatible with the type property.
     */
    value: any;

    /**
     * Check the state of this property.
     * Throw TypeError on first property with invalid type.
     */
    validate(): void;

    /**
     * Create Parameter message from this instance.
     *
     * @returns The new instance.
     */
    toParameterMessage(): rcl_interfaces.msg.Parameter;

    /**
     * Create a ParameterValue message from this instance.
     *
     * @return The new instance.
     */
    toParameterValueMessage(): rcl_interfaces.msg.ParameterValue;
  }

  /**
   * Describes a parameter.
   */
  class ParameterDescriptor {
    /**
     * Create a new instance from a parameter.
    
      * @param parameter - The parameter from which new instance is constructed. 
      * @return The new instance.
      */
    static fromParameter(parameter: Parameter): ParameterDescriptor;

    /**
     * Create new instances.

      * @param name - The descriptor name, must be a valid name. 
      * @param type - The type identifier.
      * @param description - A descriptive string.
      * @param readOnly - True indicates a parameter of this type can not be modified. Default = false.
      * @param range - An optional IntegerRange or FloatingPointRange. 
      */
    constructor(
      name: string,
      type: ParameterType,
      description?: string,
      readOnly?: boolean,
      range?: Range
    );

    /**
     * The name property.
     */

    readonly name: string;

    /**
     * The type property.
     */
    readonly type: ParameterType;

    /**
     * A descriptive string property.
     */
    readonly description: string;

    /**
     * The readOnly property.
     */
    readonly readOnly: boolean;

    /**
     * Get additionalConstraints property.
     */
    additionalConstraints: string;

    /**
     * Determine if rangeConstraint property has been set.
     *
     * @returns True if a range property is defined; falst otherwise.
     */
    hasRange(): boolean;

    /**
     * Get range property.
     */

    rangeConstraint: Range;

    /**
     * Check the state and ensure it is valid.
     * Throw a TypeError if invalid state is detected.
     */
    validate(): void;

    /**
     * Check a parameter for consistency with this descriptor.
     * Throw an Error if an inconsistent state is detected.
     *
     * @param parameter - The parameter to test for consistency.
     */
    validateParameter(parameter: Parameter): void;

    /**
     * Create a ParameterDescriptor message from this descriptor.
     *
     * @returns The new message.
     */
    toMessage(): rcl_interfaces.msg.ParameterDescriptor;
  }

  /**
   * An abstract class defining a range of numbers between 2 points inclusively
   * divided by a step value.
   * @class
   */
  abstract class Range {
    /**
     * Create a new instance.
     *
     * @param fromValue - The lowest inclusive value in range
     * @param toValue - The highest inclusive value in range
     * @param step - The internal unit size.
     */
    constructor(from: number, to: number, step?: number);

    /**
     * The lowest inclusive value in range.
     */
    readonly fromValue: number;

    /**
     * The highest inclusive value in range.
     */
    readonly toValue: number;

    /**
     * The internal unit size.
     */
    readonly step: number;

    /**
     * Determine if a value is within this range.
     * A TypeError is thrown when value is not a number.
     * Subclasses should override and call this method for basic type checking.
     *
     * @param value - The number to check.
     * @returns True if value satisfies the range; false otherwise.
     */
    inRange(value: number): boolean;

    /**
     * Abstract method that determines if a ParameterType is compatible.
     * Subclasses must implement this method.
     *
     * @param parameterType - The parameter type to test.
     * @returns True if parameterType is compatible; otherwise return false.
     */
    isValidType(parameterType: ParameterType): boolean;
  }

  /**
   * Defines a range for floating point values.
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
    constructor(from: number, to: number, step?: number, tolerance?: number);

    /**
     * Determine if a ParameterType is compatible.
     *
     * @param parameterType - The parameter type to test.
     * @returns True if parameterType is compatible; otherwise return false.
     */
    isValidType(parameterType: ParameterType): boolean;

    /**
     * Determine if a value is within this range.
     * A TypeError is thrown when value is not a number.
     *
     * @param value - The number to check.
     * @returns True if value satisfies the range; false otherwise.
     */
    inRange(value: number): boolean;
  }

  /**
   *
   */
  class IntegerRange extends FloatingPointRange {
    /**
     * Create a new instance.
     * @constructor
     * @param {number} fromValue - The lowest inclusive value in range
     * @param {number} toValue - The highest inclusive value in range
     * @param {number} step - The internal unit size.
     * @param {number} tolerance - The plus/minus tolerance for number equivalence.
     */
    constructor(from: number, to: number, step?: number, tolerance?: number);

    /**
     * Determine if a ParameterType is compatible.
     *
     * @param parameterType - The parameter type to test.
     * @returns True if parameterType is compatible; otherwise return false.
     */
    isValidType(parameterType: ParameterType): boolean;
  }
}
