# Error Handling Examples

This directory contains examples demonstrating structured error handling in rclnodejs. These examples showcase the comprehensive error class hierarchy for better debugging and error recovery patterns.

## Overview

rclnodejs provides 21 specialized error classes organized in a clear hierarchy, making it easier to catch, handle, and debug errors with rich context. All errors extend from `RclNodeError` and include properties like `code`, `nodeName`, `entityType`, `entityName`, and detailed context information.

## Error Hierarchy

```
RclNodeError (base)
├── ValidationError
│   ├── TypeValidationError
│   ├── RangeValidationError
│   └── NameValidationError
├── OperationError
│   ├── TimeoutError
│   ├── AbortError
│   ├── ServiceNotFoundError
│   └── NodeNotFoundError
├── ParameterError
│   ├── ParameterNotFoundError
│   ├── ParameterTypeError
│   └── ReadOnlyParameterError
├── TopicError
│   ├── PublisherError
│   └── SubscriptionError
├── ActionError
│   ├── GoalRejectedError
│   └── ActionServerNotFoundError
└── NativeError
```

## Examples

### 1. Type Validation (`error-handling-example.js` - Example 1)

**Purpose**: Demonstrates catching `TypeValidationError` when providing wrong argument types.

- **Functionality**: Attempts to create node with invalid type, catches and displays error details
- **Features**: Shows `argumentName`, `expectedType`, and `providedValue` properties
- **Key Properties**: `error.argumentName`, `error.expectedType`, `error.providedValue`

### 2. Range Validation (`error-handling-example.js` - Example 2)

**Purpose**: Demonstrates catching `RangeValidationError` for out-of-bounds values.

- **Functionality**: Attempts to create rate with invalid frequency (>1000 Hz), catches error
- **Features**: Shows `validationRule` and `providedValue` for constraint violations
- **Key Properties**: `error.validationRule`, `error.providedValue`, `error.nodeName`

### 3. Service Errors (`error-handling-example.js` - Example 3)

**Purpose**: Demonstrates `TimeoutError` and `AbortError` handling for service operations.

- **Functionality**: Shows timeout handling and manual request cancellation
- **Features**: Demonstrates timeout with `sendRequestAsync({ timeout })` and `AbortController`
- **Key Properties**: `error.timeout`, `error.operationType`, `error.entityName`

### 4. Publisher Errors (`error-handling-example.js` - Example 4)

**Purpose**: Demonstrates catching `PublisherError` during publisher creation.

- **Functionality**: Attempts to create publisher with invalid message type
- **Features**: Shows error handling for topic creation failures
- **Key Class**: `PublisherError`

### 5. Subscription Errors (`error-handling-example.js` - Example 5)

**Purpose**: Demonstrates catching `SubscriptionError` during subscription creation.

- **Functionality**: Attempts to create subscription with invalid message type
- **Features**: Shows error handling for subscription creation failures
- **Key Class**: `SubscriptionError`

### 6. Parameter Errors (`error-handling-example.js` - Example 6)

**Purpose**: Demonstrates `ParameterTypeError` for parameter type mismatches.

- **Functionality**: Creates parameter with wrong value type, catches type error
- **Features**: Shows parameter validation and type checking
- **Key Class**: `ParameterTypeError`

### 7. Name Validation (`error-handling-example.js` - Example 7)

**Purpose**: Demonstrates `NameValidationError` for invalid ROS names.

- **Functionality**: Validates topic name with invalid characters
- **Features**: Shows `invalidIndex` property pointing to error location
- **Key Properties**: `error.invalidIndex`, `error.message`

### 8. Error Recovery (`error-handling-example.js` - Example 8)

**Purpose**: Demonstrates retry logic with structured error handling.

- **Functionality**: Implements retry pattern with different handling for timeout vs abort
- **Features**: Shows differentiation between recoverable and non-recoverable errors
- **Pattern**: Retry on `TimeoutError`, abort on `AbortError` or other errors

### 9. Error Serialization (`error-handling-example.js` - Example 9)

**Purpose**: Demonstrates using `toJSON()` for logging and debugging.

- **Functionality**: Serializes error to JSON for structured logging
- **Features**: Shows `toJSON()` method producing complete error information
- **Method**: `error.toJSON()`

### 10. Generic Error Handler (`error-handling-example.js` - Example 10)

**Purpose**: Demonstrates reusable error handler for all rclnodejs errors.

- **Functionality**: Implements `handleRclError()` function for consistent error handling
- **Features**: Extracts common error properties, handles specific error types
- **Pattern**: Single function handling all `RclNodeError` subclasses

## How to Run

1. **Prerequisites**: Ensure ROS 2 is installed and sourced

2. **Run All Examples**:

   ```bash
   node example/error-handling/error-handling-example.js
   ```

3. **Expected Output**: Demonstrates all 10 error handling patterns with clear success indicators

## Key Concepts Demonstrated

### Error Properties

All `RclNodeError` instances include:

- `name`: Error class name (e.g., `'TypeValidationError'`)
- `message`: Human-readable error description
- `code`: Machine-readable error code (e.g., `'INVALID_TYPE'`)
- `nodeName`: Associated node name (when applicable)
- `entityType`: Entity type causing error (e.g., `'service'`, `'publisher'`)
- `entityName`: Specific entity name (e.g., topic/service name)
- `timestamp`: Error creation timestamp

### Type-Specific Properties

- **TypeValidationError**: `argumentName`, `expectedType`, `providedValue`
- **RangeValidationError**: `validationRule`, `providedValue`
- **NameValidationError**: `invalidIndex`
- **TimeoutError**: `timeout`, `operationType`
- **ParameterTypeError**: `argumentName`, `expectedType`

### Error Methods

- `toJSON()`: Serialize error to plain object for logging
- `toString()`: Format error as readable string with context

### Error Handling Patterns

- **Type Guards**: Use `instanceof` to check specific error types
- **Recovery Logic**: Different handling for recoverable vs non-recoverable errors
- **Error Chaining**: Support for `cause` property linking related errors
- **Structured Logging**: Use `toJSON()` for logging systems
- **Generic Handlers**: Single function handling multiple error types

## Usage Examples

### Basic Error Catching

```javascript
try {
  rclnodejs.createNode(123, 'namespace');
} catch (error) {
  if (error instanceof rclnodejs.TypeValidationError) {
    console.log(
      `Expected ${error.expectedType}, got ${typeof error.providedValue}`
    );
  }
}
```

### Service Timeout Handling

```javascript
try {
  await client.sendRequestAsync(request, { timeout: 5000 });
} catch (error) {
  if (error instanceof rclnodejs.TimeoutError) {
    console.log(`Timeout after ${error.timeout}ms`);
  } else if (error instanceof rclnodejs.AbortError) {
    console.log('Request cancelled');
  }
}
```

### Generic Error Handler

```javascript
function handleRclError(error, operation) {
  if (!(error instanceof rclnodejs.RclNodeError)) {
    console.error(`Unexpected error: ${error.message}`);
    return;
  }

  const context = [];
  if (error.nodeName) context.push(`node: ${error.nodeName}`);
  if (error.entityName)
    context.push(`${error.entityType}: ${error.entityName}`);

  console.error(`${error.name} in ${operation}: ${error.message}`);
}
```

## Notes

- All error classes extend `RclNodeError` for consistent `instanceof` checks
- Error context properties are populated based on the operation
- TypeScript definitions provide full type safety for error handling
- Use `toJSON()` for integration with logging frameworks
- Error codes are machine-readable constants for programmatic handling
- Backward compatibility maintained - existing error handling continues to work
