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

/**
 * Base error class for all rclnodejs errors.
 * Provides structured error information with context.
 * @class
 */
class RclNodeError extends Error {
  /**
   * @param {string} message - Human-readable error message
   * @param {object} [options] - Additional error context
   * @param {string} [options.code] - Machine-readable error code (e.g., 'TIMEOUT', 'INVALID_ARGUMENT')
   * @param {string} [options.nodeName] - Name of the node where error occurred
   * @param {string} [options.entityType] - Type of entity (publisher, subscription, client, etc.)
   * @param {string} [options.entityName] - Name of the entity (topic name, service name, etc.)
   * @param {Error} [options.cause] - Original error that caused this error
   * @param {any} [options.details] - Additional error-specific details
   */
  constructor(message, options = {}) {
    super(message);

    // Maintains proper stack trace for where our error was thrown
    Error.captureStackTrace(this, this.constructor);

    this.name = this.constructor.name;
    this.code = options.code || 'UNKNOWN_ERROR';
    this.nodeName = options.nodeName;
    this.entityType = options.entityType;
    this.entityName = options.entityName;
    this.details = options.details;

    // Error chaining (ES2022 feature, Node.js 16.9+)
    if (options.cause) {
      this.cause = options.cause;
    }

    // Timestamp for logging/debugging
    this.timestamp = new Date();
  }

  /**
   * Returns a detailed error object for logging/serialization
   * @return {object} Structured error information
   */
  toJSON() {
    return {
      name: this.name,
      message: this.message,
      code: this.code,
      nodeName: this.nodeName,
      entityType: this.entityType,
      entityName: this.entityName,
      details: this.details,
      timestamp: this.timestamp.toISOString(),
      stack: this.stack,
      cause: this.cause
        ? this.cause.toJSON?.() || this.cause.message
        : undefined,
    };
  }

  /**
   * Returns a user-friendly error description
   * @return {string} Formatted error string
   */
  toString() {
    let str = `${this.name}: ${this.message}`;
    if (this.code) str += ` [${this.code}]`;
    if (this.nodeName) str += ` (node: ${this.nodeName})`;
    if (this.entityName)
      str += ` (${this.entityType || 'entity'}: ${this.entityName})`;
    return str;
  }
}

/**
 * Error thrown when validation fails
 * @class
 * @extends RclNodeError
 */
class ValidationError extends RclNodeError {
  /**
   * @param {string} message - Error message
   * @param {object} [options] - Additional options
   * @param {string} [options.argumentName] - Name of the argument that failed validation
   * @param {any} [options.providedValue] - The value that was provided
   * @param {string} [options.expectedType] - The expected type or format
   * @param {string} [options.validationRule] - The validation rule that failed
   */
  constructor(message, options = {}) {
    super(message, { code: 'VALIDATION_ERROR', ...options });
    this.argumentName = options.argumentName;
    this.providedValue = options.providedValue;
    this.expectedType = options.expectedType;
    this.validationRule = options.validationRule;
  }
}

/**
 * Type validation error
 * @class
 * @extends ValidationError
 */
class TypeValidationError extends ValidationError {
  /**
   * @param {string} argumentName - Name of the argument
   * @param {any} providedValue - The value that was provided
   * @param {string} expectedType - The expected type
   * @param {object} [options] - Additional options
   */
  constructor(argumentName, providedValue, expectedType, options = {}) {
    super(
      `Invalid type for '${argumentName}': expected ${expectedType}, got ${typeof providedValue}`,
      {
        code: 'INVALID_TYPE',
        argumentName,
        providedValue,
        expectedType,
        ...options,
      }
    );
  }
}

/**
 * Range/value validation error
 * @class
 * @extends ValidationError
 */
class RangeValidationError extends ValidationError {
  /**
   * @param {string} argumentName - Name of the argument
   * @param {any} providedValue - The value that was provided
   * @param {string} constraint - The constraint that was violated
   * @param {object} [options] - Additional options
   */
  constructor(argumentName, providedValue, constraint, options = {}) {
    super(
      `Value '${providedValue}' for '${argumentName}' is out of range: ${constraint}`,
      {
        code: 'OUT_OF_RANGE',
        argumentName,
        providedValue,
        validationRule: constraint,
        ...options,
      }
    );
  }
}

/**
 * ROS name validation error (topics, nodes, services)
 * @class
 * @extends ValidationError
 */
class NameValidationError extends ValidationError {
  /**
   * @param {string} name - The invalid name
   * @param {string} nameType - Type of name (node, topic, service, etc.)
   * @param {string} validationResult - The validation error message
   * @param {number} invalidIndex - Index where validation failed
   * @param {object} [options] - Additional options
   */
  constructor(name, nameType, validationResult, invalidIndex, options = {}) {
    super(
      `Invalid ${nameType} name '${name}': ${validationResult}` +
        (invalidIndex >= 0 ? ` at index ${invalidIndex}` : ''),
      {
        code: 'INVALID_NAME',
        argumentName: nameType,
        providedValue: name,
        details: { validationResult, invalidIndex },
        ...options,
      }
    );
    this.invalidIndex = invalidIndex;
    this.validationResult = validationResult;
  }
}

/**
 * Base class for operation/runtime errors
 * @class
 * @extends RclNodeError
 */
class OperationError extends RclNodeError {
  /**
   * @param {string} message - Error message
   * @param {object} [options] - Additional options
   */
  constructor(message, options = {}) {
    super(message, { code: 'OPERATION_ERROR', ...options });
  }
}

/**
 * Request timeout error
 * @class
 * @extends OperationError
 */
class TimeoutError extends OperationError {
  /**
   * @param {string} operationType - Type of operation that timed out
   * @param {number} timeoutMs - Timeout duration in milliseconds
   * @param {object} [options] - Additional options
   */
  constructor(operationType, timeoutMs, options = {}) {
    super(`${operationType} timeout after ${timeoutMs}ms`, {
      code: 'TIMEOUT',
      details: { timeoutMs, operationType },
      ...options,
    });
    this.timeout = timeoutMs;
    this.operationType = operationType;
  }
}

/**
 * Request abortion error
 * @class
 * @extends OperationError
 */
class AbortError extends OperationError {
  /**
   * @param {string} operationType - Type of operation that was aborted
   * @param {string} [reason] - Reason for abortion
   * @param {object} [options] - Additional options
   */
  constructor(operationType, reason, options = {}) {
    super(`${operationType} was aborted` + (reason ? `: ${reason}` : ''), {
      code: 'ABORTED',
      details: { operationType, reason },
      ...options,
    });
    this.operationType = operationType;
    this.abortReason = reason;
  }
}

/**
 * Service not available error
 * @class
 * @extends OperationError
 */
class ServiceNotFoundError extends OperationError {
  /**
   * @param {string} serviceName - Name of the service
   * @param {object} [options] - Additional options
   */
  constructor(serviceName, options = {}) {
    super(`Service '${serviceName}' is not available`, {
      code: 'SERVICE_NOT_FOUND',
      entityType: 'service',
      entityName: serviceName,
      ...options,
    });
    this.serviceName = serviceName;
  }
}

/**
 * Remote node not found error
 * @class
 * @extends OperationError
 */
class NodeNotFoundError extends OperationError {
  /**
   * @param {string} nodeName - Name of the node
   * @param {object} [options] - Additional options
   */
  constructor(nodeName, options = {}) {
    super(`Node '${nodeName}' not found or not available`, {
      code: 'NODE_NOT_FOUND',
      entityType: 'node',
      entityName: nodeName,
      ...options,
    });
    this.targetNodeName = nodeName;
  }
}

/**
 * Base error for parameter operations
 * @class
 * @extends RclNodeError
 */
class ParameterError extends RclNodeError {
  /**
   * @param {string} message - Error message
   * @param {string} parameterName - Name of the parameter
   * @param {object} [options] - Additional options
   */
  constructor(message, parameterName, options = {}) {
    super(message, {
      code: 'PARAMETER_ERROR',
      entityType: 'parameter',
      entityName: parameterName,
      ...options,
    });
    this.parameterName = parameterName;
  }
}

/**
 * Parameter not found error
 * @class
 * @extends ParameterError
 */
class ParameterNotFoundError extends ParameterError {
  /**
   * @param {string} parameterName - Name of the parameter
   * @param {string} nodeName - Name of the node
   * @param {object} [options] - Additional options
   */
  constructor(parameterName, nodeName, options = {}) {
    super(
      `Parameter '${parameterName}' not found on node '${nodeName}'`,
      parameterName,
      {
        code: 'PARAMETER_NOT_FOUND',
        nodeName,
        ...options,
      }
    );
  }
}

/**
 * Parameter type mismatch error
 * @class
 * @extends ParameterError
 */
class ParameterTypeError extends ParameterError {
  /**
   * @param {string} parameterName - Name of the parameter
   * @param {string} expectedType - Expected parameter type
   * @param {string} actualType - Actual parameter type
   * @param {object} [options] - Additional options
   */
  constructor(parameterName, expectedType, actualType, options = {}) {
    super(
      `Type mismatch for parameter '${parameterName}': expected ${expectedType}, got ${actualType}`,
      parameterName,
      {
        code: 'PARAMETER_TYPE_MISMATCH',
        details: { expectedType, actualType },
        ...options,
      }
    );
    this.expectedType = expectedType;
    this.actualType = actualType;
  }
}

/**
 * Read-only parameter modification error
 * @class
 * @extends ParameterError
 */
class ReadOnlyParameterError extends ParameterError {
  /**
   * @param {string} parameterName - Name of the parameter
   * @param {object} [options] - Additional options
   */
  constructor(parameterName, options = {}) {
    super(
      `Cannot modify read-only parameter '${parameterName}'`,
      parameterName,
      {
        code: 'PARAMETER_READ_ONLY',
        ...options,
      }
    );
  }
}

/**
 * Base error for topic operations
 * @class
 * @extends RclNodeError
 */
class TopicError extends RclNodeError {
  /**
   * @param {string} message - Error message
   * @param {string} topicName - Name of the topic
   * @param {object} [options] - Additional options
   */
  constructor(message, topicName, options = {}) {
    super(message, {
      code: 'TOPIC_ERROR',
      entityType: 'topic',
      entityName: topicName,
      ...options,
    });
    this.topicName = topicName;
  }
}

/**
 * Publisher-specific error
 * @class
 * @extends TopicError
 */
class PublisherError extends TopicError {
  /**
   * @param {string} message - Error message
   * @param {string} topicName - Name of the topic
   * @param {object} [options] - Additional options
   */
  constructor(message, topicName, options = {}) {
    super(message, topicName, {
      code: 'PUBLISHER_ERROR',
      entityType: 'publisher',
      ...options,
    });
  }
}

/**
 * Subscription-specific error
 * @class
 * @extends TopicError
 */
class SubscriptionError extends TopicError {
  /**
   * @param {string} message - Error message
   * @param {string} topicName - Name of the topic
   * @param {object} [options] - Additional options
   */
  constructor(message, topicName, options = {}) {
    super(message, topicName, {
      code: 'SUBSCRIPTION_ERROR',
      entityType: 'subscription',
      ...options,
    });
  }
}

/**
 * Base error for action operations
 * @class
 * @extends RclNodeError
 */
class ActionError extends RclNodeError {
  /**
   * @param {string} message - Error message
   * @param {string} actionName - Name of the action
   * @param {object} [options] - Additional options
   */
  constructor(message, actionName, options = {}) {
    super(message, {
      code: 'ACTION_ERROR',
      entityType: 'action',
      entityName: actionName,
      ...options,
    });
    this.actionName = actionName;
  }
}

/**
 * Goal rejected by action server
 * @class
 * @extends ActionError
 */
class GoalRejectedError extends ActionError {
  /**
   * @param {string} actionName - Name of the action
   * @param {string} goalId - ID of the rejected goal
   * @param {object} [options] - Additional options
   */
  constructor(actionName, goalId, options = {}) {
    super(`Goal rejected by action server '${actionName}'`, actionName, {
      code: 'GOAL_REJECTED',
      details: { goalId },
      ...options,
    });
    this.goalId = goalId;
  }
}

/**
 * Action server not found
 * @class
 * @extends ActionError
 */
class ActionServerNotFoundError extends ActionError {
  /**
   * @param {string} actionName - Name of the action
   * @param {object} [options] - Additional options
   */
  constructor(actionName, options = {}) {
    super(`Action server '${actionName}' is not available`, actionName, {
      code: 'ACTION_SERVER_NOT_FOUND',
      ...options,
    });
  }
}

/**
 * Wraps errors from native C++ layer with additional context
 * @class
 * @extends RclNodeError
 */
class NativeError extends RclNodeError {
  /**
   * @param {string} nativeMessage - Error message from C++ layer
   * @param {string} operation - Operation that failed
   * @param {object} [options] - Additional options
   */
  constructor(nativeMessage, operation, options = {}) {
    super(`Native operation failed: ${operation} - ${nativeMessage}`, {
      code: 'NATIVE_ERROR',
      details: { nativeMessage, operation },
      ...options,
    });
    this.nativeMessage = nativeMessage;
    this.operation = operation;
  }
}

module.exports = {
  // Base error
  RclNodeError,

  // Validation errors
  ValidationError,
  TypeValidationError,
  RangeValidationError,
  NameValidationError,

  // Operation errors
  OperationError,
  TimeoutError,
  AbortError,
  ServiceNotFoundError,
  NodeNotFoundError,

  // Parameter errors
  ParameterError,
  ParameterNotFoundError,
  ParameterTypeError,
  ReadOnlyParameterError,

  // Topic errors
  TopicError,
  PublisherError,
  SubscriptionError,

  // Action errors
  ActionError,
  GoalRejectedError,
  ActionServerNotFoundError,

  // Native error
  NativeError,
};
