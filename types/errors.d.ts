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
   * Options for RclNodeError constructor
   */
  export interface RclNodeErrorOptions {
    /** Machine-readable error code (e.g., 'TIMEOUT', 'INVALID_ARGUMENT') */
    code?: string;
    /** Name of the node where error occurred */
    nodeName?: string;
    /** Type of entity (publisher, subscription, client, etc.) */
    entityType?: string;
    /** Name of the entity (topic name, service name, etc.) */
    entityName?: string;
    /** Original error that caused this error */
    cause?: Error;
    /** Additional error-specific details */
    details?: any;
  }

  /**
   * Base error class for all rclnodejs errors.
   * Provides structured error information with context.
   */
  export class RclNodeError extends Error {
    /** Error code for machine-readable error identification */
    code: string;
    /** Name of the node where error occurred */
    nodeName?: string;
    /** Type of entity (publisher, subscription, client, etc.) */
    entityType?: string;
    /** Name of the entity (topic name, service name, etc.) */
    entityName?: string;
    /** Additional error-specific details */
    details?: any;
    /** Original error that caused this error */
    cause?: Error;
    /** Timestamp when error was created */
    timestamp: Date;

    /**
     * @param message - Human-readable error message
     * @param options - Additional error context
     */
    constructor(message: string, options?: RclNodeErrorOptions);

    /**
     * Returns a detailed error object for logging/serialization
     */
    toJSON(): {
      name: string;
      message: string;
      code: string;
      nodeName?: string;
      entityType?: string;
      entityName?: string;
      details?: any;
      timestamp: string;
      stack?: string;
      cause?: any;
    };

    /**
     * Returns a user-friendly error description
     */
    toString(): string;
  }

  /**
   * Options for ValidationError constructor
   */
  export interface ValidationErrorOptions extends RclNodeErrorOptions {
    /** Name of the argument that failed validation */
    argumentName?: string;
    /** The value that was provided */
    providedValue?: any;
    /** The expected type or format */
    expectedType?: string;
    /** The validation rule that failed */
    validationRule?: string;
  }

  /**
   * Error thrown when validation fails
   */
  export class ValidationError extends RclNodeError {
    /** Name of the argument that failed validation */
    argumentName?: string;
    /** The value that was provided */
    providedValue?: any;
    /** The expected type or format */
    expectedType?: string;
    /** The validation rule that failed */
    validationRule?: string;

    /**
     * @param message - Error message
     * @param options - Additional options
     */
    constructor(message: string, options?: ValidationErrorOptions);
  }

  /**
   * Type validation error
   */
  export class TypeValidationError extends ValidationError {
    /**
     * @param argumentName - Name of the argument
     * @param providedValue - The value that was provided
     * @param expectedType - The expected type
     * @param options - Additional options
     */
    constructor(
      argumentName: string,
      providedValue: any,
      expectedType: string,
      options?: RclNodeErrorOptions
    );
  }

  /**
   * Range/value validation error
   */
  export class RangeValidationError extends ValidationError {
    /**
     * @param argumentName - Name of the argument
     * @param providedValue - The value that was provided
     * @param constraint - The constraint that was violated
     * @param options - Additional options
     */
    constructor(
      argumentName: string,
      providedValue: any,
      constraint: string,
      options?: RclNodeErrorOptions
    );
  }

  /**
   * ROS name validation error (topics, nodes, services)
   */
  export class NameValidationError extends ValidationError {
    /** Index where validation failed */
    invalidIndex: number;
    /** The validation error message */
    validationResult: string;

    /**
     * @param name - The invalid name
     * @param nameType - Type of name (node, topic, service, etc.)
     * @param validationResult - The validation error message
     * @param invalidIndex - Index where validation failed
     * @param options - Additional options
     */
    constructor(
      name: string,
      nameType: string,
      validationResult: string,
      invalidIndex: number,
      options?: RclNodeErrorOptions
    );
  }

  /**
   * Base class for operation/runtime errors
   */
  export class OperationError extends RclNodeError {
    /**
     * @param message - Error message
     * @param options - Additional options
     */
    constructor(message: string, options?: RclNodeErrorOptions);
  }

  /**
   * Request timeout error
   */
  export class TimeoutError extends OperationError {
    /** Timeout duration in milliseconds */
    timeout: number;
    /** Type of operation that timed out */
    operationType: string;

    /**
     * @param operationType - Type of operation that timed out
     * @param timeoutMs - Timeout duration in milliseconds
     * @param options - Additional options
     */
    constructor(
      operationType: string,
      timeoutMs: number,
      options?: RclNodeErrorOptions
    );
  }

  /**
   * Request abortion error
   */
  export class AbortError extends OperationError {
    /** Type of operation that was aborted */
    operationType: string;
    /** Reason for abortion */
    abortReason?: string;

    /**
     * @param operationType - Type of operation that was aborted
     * @param reason - Reason for abortion
     * @param options - Additional options
     */
    constructor(
      operationType: string,
      reason?: string,
      options?: RclNodeErrorOptions
    );
  }

  /**
   * Service not available error
   */
  export class ServiceNotFoundError extends OperationError {
    /** Name of the service */
    serviceName: string;

    /**
     * @param serviceName - Name of the service
     * @param options - Additional options
     */
    constructor(serviceName: string, options?: RclNodeErrorOptions);
  }

  /**
   * Remote node not found error
   */
  export class NodeNotFoundError extends OperationError {
    /** Name of the target node */
    targetNodeName: string;

    /**
     * @param nodeName - Name of the node
     * @param options - Additional options
     */
    constructor(nodeName: string, options?: RclNodeErrorOptions);
  }

  /**
   * Base error for parameter operations
   */
  export class ParameterError extends RclNodeError {
    /** Name of the parameter */
    parameterName: string;

    /**
     * @param message - Error message
     * @param parameterName - Name of the parameter
     * @param options - Additional options
     */
    constructor(
      message: string,
      parameterName: string,
      options?: RclNodeErrorOptions
    );
  }

  /**
   * Parameter not found error
   */
  export class ParameterNotFoundError extends ParameterError {
    /**
     * @param parameterName - Name of the parameter
     * @param nodeName - Name of the node
     * @param options - Additional options
     */
    constructor(
      parameterName: string,
      nodeName: string,
      options?: RclNodeErrorOptions
    );
  }

  /**
   * Parameter type mismatch error
   */
  export class ParameterTypeError extends ParameterError {
    /** Expected parameter type */
    expectedType: string;
    /** Actual parameter type */
    actualType: string;

    /**
     * @param parameterName - Name of the parameter
     * @param expectedType - Expected parameter type
     * @param actualType - Actual parameter type
     * @param options - Additional options
     */
    constructor(
      parameterName: string,
      expectedType: string,
      actualType: string,
      options?: RclNodeErrorOptions
    );
  }

  /**
   * Read-only parameter modification error
   */
  export class ReadOnlyParameterError extends ParameterError {
    /**
     * @param parameterName - Name of the parameter
     * @param options - Additional options
     */
    constructor(parameterName: string, options?: RclNodeErrorOptions);
  }

  /**
   * Base error for topic operations
   */
  export class TopicError extends RclNodeError {
    /** Name of the topic */
    topicName: string;

    /**
     * @param message - Error message
     * @param topicName - Name of the topic
     * @param options - Additional options
     */
    constructor(
      message: string,
      topicName: string,
      options?: RclNodeErrorOptions
    );
  }

  /**
   * Publisher-specific error
   */
  export class PublisherError extends TopicError {
    /**
     * @param message - Error message
     * @param topicName - Name of the topic
     * @param options - Additional options
     */
    constructor(
      message: string,
      topicName: string,
      options?: RclNodeErrorOptions
    );
  }

  /**
   * Subscription-specific error
   */
  export class SubscriptionError extends TopicError {
    /**
     * @param message - Error message
     * @param topicName - Name of the topic
     * @param options - Additional options
     */
    constructor(
      message: string,
      topicName: string,
      options?: RclNodeErrorOptions
    );
  }

  /**
   * Base error for action operations
   */
  export class ActionError extends RclNodeError {
    /** Name of the action */
    actionName: string;

    /**
     * @param message - Error message
     * @param actionName - Name of the action
     * @param options - Additional options
     */
    constructor(
      message: string,
      actionName: string,
      options?: RclNodeErrorOptions
    );
  }

  /**
   * Goal rejected by action server
   */
  export class GoalRejectedError extends ActionError {
    /** ID of the rejected goal */
    goalId: string;

    /**
     * @param actionName - Name of the action
     * @param goalId - ID of the rejected goal
     * @param options - Additional options
     */
    constructor(
      actionName: string,
      goalId: string,
      options?: RclNodeErrorOptions
    );
  }

  /**
   * Action server not found
   */
  export class ActionServerNotFoundError extends ActionError {
    /**
     * @param actionName - Name of the action
     * @param options - Additional options
     */
    constructor(actionName: string, options?: RclNodeErrorOptions);
  }

  /**
   * Wraps errors from native C++ layer with additional context
   */
  export class NativeError extends RclNodeError {
    /** Error message from C++ layer */
    nativeMessage: string;
    /** Operation that failed */
    operation: string;

    /**
     * @param nativeMessage - Error message from C++ layer
     * @param operation - Operation that failed
     * @param options - Additional options
     */
    constructor(
      nativeMessage: string,
      operation: string,
      options?: RclNodeErrorOptions
    );
  }
}
