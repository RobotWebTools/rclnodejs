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

declare module 'rclnodejs' {
  /**
   * Validation issue problem types
   */
  export const ValidationProblem: {
    /** Field exists in object but not in message schema */
    readonly UNKNOWN_FIELD: 'UNKNOWN_FIELD';
    /** Field type doesn't match expected type */
    readonly TYPE_MISMATCH: 'TYPE_MISMATCH';
    /** Required field is missing */
    readonly MISSING_FIELD: 'MISSING_FIELD';
    /** Array length constraint violated */
    readonly ARRAY_LENGTH: 'ARRAY_LENGTH';
    /** Value is out of valid range */
    readonly OUT_OF_RANGE: 'OUT_OF_RANGE';
    /** Nested message validation failed */
    readonly NESTED_ERROR: 'NESTED_ERROR';
  };

  /**
   * Field type information from message schema
   */
  export interface MessageFieldType {
    /** The type name (e.g., 'string', 'float64', 'Vector3') */
    type: string;
    /** Whether this is a primitive ROS type */
    isPrimitiveType: boolean;
    /** Whether this field is an array */
    isArray: boolean;
    /** For fixed-size arrays, the required size */
    arraySize?: number;
    /** Whether this is a fixed-size array */
    isFixedSizeArray: boolean;
    /** Whether this has an upper bound (bounded sequence) */
    isUpperBound: boolean;
    /** Whether this is a dynamic array */
    isDynamicArray: boolean;
    /** Package name for non-primitive types */
    pkgName?: string;
    /** String upper bound for bounded strings */
    stringUpperBound?: number;
  }

  /**
   * Field definition from message schema
   */
  export interface MessageField {
    /** Field name */
    name: string;
    /** Field type information */
    type: MessageFieldType;
    /** Default value if any */
    default_value?: any;
  }

  /**
   * Constant definition from message schema
   */
  export interface MessageConstant {
    /** Constant name */
    name: string;
    /** Constant type */
    type: string;
    /** Constant value */
    value: any;
  }

  /**
   * Message schema definition
   */
  export interface MessageSchema {
    /** Array of field definitions */
    fields: MessageField[];
    /** Array of constant definitions */
    constants: MessageConstant[];
    /** Full message type string (e.g., 'std_msgs/msg/String') */
    messageType: string;
    /** Base type information */
    baseType?: {
      pkgName: string;
      type: string;
      isPrimitiveType: boolean;
    };
  }

  /**
   * Options for message validation
   */
  export interface MessageValidationOptions {
    /** If true, unknown fields cause validation failure (default: false) */
    strict?: boolean;
    /** If true, validate field types (default: true) */
    checkTypes?: boolean;
    /** If true, check for missing fields (default: false) */
    checkRequired?: boolean;
  }

  /**
   * Result of message validation
   */
  export interface MessageValidationResult {
    /** Whether the message is valid */
    valid: boolean;
    /** Array of validation issues found */
    issues: MessageValidationIssue[];
  }

  /**
   * Get the schema definition for a message type
   * @param typeClass - Message type class or identifier
   * @returns Schema definition with fields and constants, or null if not found
   */
  export function getMessageSchema(typeClass: TypeClass): MessageSchema | null;

  /**
   * Get field names for a message type
   * @param typeClass - Message type class or identifier
   * @returns Array of field names
   */
  export function getFieldNames(typeClass: TypeClass): string[];

  /**
   * Get type information for a specific field
   * @param typeClass - Message type class or identifier
   * @param fieldName - Name of the field
   * @returns Field type information or null if not found
   */
  export function getFieldType(
    typeClass: TypeClass,
    fieldName: string
  ): MessageFieldType | null;

  /**
   * Validate a message object against its schema
   * @param obj - Plain object to validate
   * @param typeClass - Message type class or identifier
   * @param options - Validation options
   * @returns Validation result with valid flag and issues array
   */
  export function validateMessage(
    obj: any,
    typeClass: TypeClass,
    options?: MessageValidationOptions
  ): MessageValidationResult;

  /**
   * Validate a message and throw if invalid
   * @param obj - Plain object to validate
   * @param typeClass - Message type class or identifier
   * @param options - Validation options
   * @throws MessageValidationError if validation fails
   */
  export function assertValidMessage(
    obj: any,
    typeClass: TypeClass,
    options?: MessageValidationOptions
  ): void;

  /**
   * Create a validator function for a specific message type
   * @param typeClass - Message type class or identifier
   * @param defaultOptions - Default validation options
   * @returns Validator function that takes (obj, options?) and returns validation result
   */
  export function createMessageValidator(
    typeClass: TypeClass,
    defaultOptions?: MessageValidationOptions
  ): (obj: any, options?: MessageValidationOptions) => MessageValidationResult;
}
