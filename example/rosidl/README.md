# ROS IDL (ROSIDL) Parser Examples

This directory contains examples demonstrating ROSIDL parsing capabilities using rclnodejs. ROSIDL (ROS Interface Definition Language) is used to define message, service, and action interfaces in ROS 2. These examples show how to programmatically parse and inspect ROS 2 interface definitions.

## Overview

ROSIDL parsing provides capabilities for:

- **Interface Introspection**: Programmatically examine message, service, and action definitions
- **Dynamic Analysis**: Understand interface structures at runtime
- **Code Generation**: Support for automatic code generation tools
- **Validation**: Verify interface compatibility and structure
- **Development Tools**: Build IDEs, debuggers, and analysis tools

ROSIDL parsing is useful for:

- Development tools and IDEs
- Dynamic message handling
- Interface validation and testing
- Code generation and automation
- Runtime interface discovery
- Bridge implementations between different systems

## ROSIDL Examples

### 1. Message Parsing (`rosidl-parse-msg-example.js`)

**Purpose**: Demonstrates parsing ROS 2 message definition files (.msg).

#### Functionality

This example shows how to:

- Parse a standard ROS 2 message file
- Extract message name and package information
- Analyze field definitions and their types
- Understand message structure programmatically

#### Target Message

- **Package**: `std_msgs`
- **Message**: `ColorRGBA.msg`
- **File Path**: `$AMENT_PREFIX_PATH/share/std_msgs/msg/ColorRGBA.msg`

#### Message Structure

The `ColorRGBA` message contains:

```
float32 r  # Red component
float32 g  # Green component
float32 b  # Blue component
float32 a  # Alpha (transparency) component
```

#### Features Demonstrated

- **Environment Integration**: Uses `AMENT_PREFIX_PATH` to locate ROS packages
- **Message Parsing**: Using `parseMessageFile()` method
- **Field Analysis**: Iterating through message fields
- **Type Information**: Accessing field types and names

#### Run Command

```bash
node rosidl-parse-msg-example.js
```

#### Expected Output

```
msg name: ColorRGBA
fields includes:
{
  name: 'r',
  type: 'float32',
  isArray: false,
  arraySize: 0,
  isUpperBound: false,
  defaultValue: null
}
{
  name: 'g',
  type: 'float32',
  isArray: false,
  arraySize: 0,
  isUpperBound: false,
  defaultValue: null
}
{
  name: 'b',
  type: 'float32',
  isArray: false,
  arraySize: 0,
  isUpperBound: false,
  defaultValue: null
}
{
  name: 'a',
  type: 'float32',
  isArray: false,
  arraySize: 0,
  isUpperBound: false,
  defaultValue: null
}
```

### 2. Service Parsing (`rosidl-parse-srv-example.js`)

**Purpose**: Demonstrates parsing ROS 2 service definition files (.srv).

#### Functionality

This example shows how to:

- Parse ROS 2 service interface definitions
- Extract service name and package information
- Analyze request and response field structures
- Understand service interface contracts

#### Target Service

- **Package**: `std_srvs`
- **Service**: `SetBool.srv`
- **File Path**: `$AMENT_PREFIX_PATH/share/std_srvs/srv/SetBool.srv`

#### Service Structure

The `SetBool` service contains:

```
# Request
bool data  # Input boolean value
---
# Response
bool success  # Indicates success/failure
string message  # Informational message
```

#### Features Demonstrated

- **Service Parsing**: Using `parseServiceFile()` method
- **Request/Response Analysis**: Separate handling of request and response parts
- **Package Information**: Extracting package and service names
- **Field Iteration**: Processing both request and response fields

#### Run Command

```bash
node rosidl-parse-srv-example.js
```

#### Expected Output

```
srv name: SetBool
pkg name: std_srvs
srv request fields includes:
{
  name: 'data',
  type: 'bool',
  isArray: false,
  arraySize: 0,
  isUpperBound: false,
  defaultValue: null
}
srv response fields includes:
{
  name: 'success',
  type: 'bool',
  isArray: false,
  arraySize: 0,
  isUpperBound: false,
  defaultValue: null
}
{
  name: 'message',
  type: 'string',
  isArray: false,
  arraySize: 0,
  isUpperBound: false,
  defaultValue: null
}
```

### 3. Action Parsing (`rosidl-parse-action-example.js`)

**Purpose**: Demonstrates parsing ROS 2 action definition files (.action).

#### Functionality

This example shows how to:

- Parse ROS 2 action interface definitions
- Extract action name and package information
- Analyze goal, result, and feedback field structures
- Understand complete action interface specifications

#### Target Action

- **Package**: `test_msgs`
- **Action**: `Fibonacci.action`
- **File Path**: `$AMENT_PREFIX_PATH/share/test_msgs/action/Fibonacci.action`

#### Action Structure

The `Fibonacci` action contains:

```
# Goal
int32 order  # Fibonacci sequence order
---
# Result
int32[] sequence  # Complete Fibonacci sequence
---
# Feedback
int32[] partial_sequence  # Partial sequence (feedback)
```

#### Features Demonstrated

- **Action Parsing**: Using `parseActionFile()` method
- **Three-Part Analysis**: Handling goal, result, and feedback sections
- **Array Types**: Processing array field definitions
- **Complex Structures**: Understanding multi-part action interfaces

#### Run Command

```bash
node rosidl-parse-action-example.js
```

#### Expected Output

```
action name: Fibonacci
pkg name: test_msgs
action goal fields includes:
{
  name: 'order',
  type: 'int32',
  isArray: false,
  arraySize: 0,
  isUpperBound: false,
  defaultValue: null
}
action result fields includes:
{
  name: 'sequence',
  type: 'int32',
  isArray: true,
  arraySize: 0,
  isUpperBound: false,
  defaultValue: null
}
action feedback fields includes:
{
  name: 'partial_sequence',
  type: 'int32',
  isArray: true,
  arraySize: 0,
  isUpperBound: false,
  defaultValue: null
}
```

## Field Structure Analysis

### Field Object Properties

Each parsed field contains these properties:

- **`name`**: Field name as defined in the interface
- **`type`**: ROS 2 primitive or complex type (e.g., `int32`, `string`, `float64`)
- **`isArray`**: Boolean indicating if field is an array
- **`arraySize`**: Array size (0 for dynamic arrays)
- **`isUpperBound`**: Whether array size is an upper bound
- **`defaultValue`**: Default value if specified (usually null)

### Common ROS 2 Types

- **Primitive Types**: `bool`, `int8`, `int16`, `int32`, `int64`, `uint8`, `uint16`, `uint32`, `uint64`, `float32`, `float64`, `string`
- **Array Types**: `int32[]`, `string[]`, `float64[]`
- **Bounded Arrays**: `int32[10]`, `string[5]`
- **Complex Types**: Custom message types from other packages

## ROSIDL Parser API

### Message Parsing

```javascript
const parser = require('../rosidl_parser/rosidl_parser.js');

parser.parseMessageFile(packageName, packagePath).then((spec) => {
  console.log(`Message: ${spec.msgName}`);
  spec.fields.forEach((field) => {
    console.log(`Field: ${field.name} (${field.type})`);
  });
});
```

### Service Parsing

```javascript
parser.parseServiceFile(packageName, packagePath).then((spec) => {
  console.log(`Service: ${spec.srvName}`);
  console.log('Request fields:', spec.request.fields);
  console.log('Response fields:', spec.response.fields);
});
```

### Action Parsing

```javascript
parser.parseActionFile(packageName, packagePath).then((spec) => {
  console.log(`Action: ${spec.actionName}`);
  console.log('Goal fields:', spec.goal.fields);
  console.log('Result fields:', spec.result.fields);
  console.log('Feedback fields:', spec.feedback.fields);
});
```

## Environment Setup

### Prerequisites

1. **ROS 2 Installation**: Properly installed and sourced ROS 2 environment
2. **AMENT_PREFIX_PATH**: Environment variable pointing to ROS 2 package locations
3. **Package Availability**: Target packages (`std_msgs`, `std_srvs`, `test_msgs`) must be installed

### Verification

Check your environment setup:

```bash
echo $AMENT_PREFIX_PATH
ls $AMENT_PREFIX_PATH/share/std_msgs/msg/
ls $AMENT_PREFIX_PATH/share/std_srvs/srv/
ls $AMENT_PREFIX_PATH/share/test_msgs/action/
```

## Advanced Usage Patterns

### Dynamic Interface Discovery

```javascript
const fs = require('fs');
const path = require('path');

// Find all message files in a package
function findMessageFiles(packagePath) {
  const msgDir = path.join(packagePath, 'msg');
  if (fs.existsSync(msgDir)) {
    return fs
      .readdirSync(msgDir)
      .filter((file) => file.endsWith('.msg'))
      .map((file) => path.join(msgDir, file));
  }
  return [];
}

// Parse all messages in a package
async function parseAllMessages(packageName) {
  const packagePath = path.join(
    process.env.AMENT_PREFIX_PATH,
    'share',
    packageName
  );
  const msgFiles = findMessageFiles(packagePath);

  for (const msgFile of msgFiles) {
    try {
      const spec = await parser.parseMessageFile(packageName, msgFile);
      console.log(`Parsed: ${spec.msgName}`);
    } catch (error) {
      console.error(`Failed to parse ${msgFile}:`, error);
    }
  }
}
```

### Interface Validation

```javascript
function validateMessageStructure(spec, expectedFields) {
  const actualFields = spec.fields.map((f) => ({ name: f.name, type: f.type }));

  for (const expected of expectedFields) {
    const actual = actualFields.find((f) => f.name === expected.name);
    if (!actual) {
      throw new Error(`Missing field: ${expected.name}`);
    }
    if (actual.type !== expected.type) {
      throw new Error(
        `Type mismatch for ${expected.name}: expected ${expected.type}, got ${actual.type}`
      );
    }
  }

  return true;
}

// Usage
parser.parseMessageFile('std_msgs', messagePath).then((spec) => {
  const expectedFields = [
    { name: 'r', type: 'float32' },
    { name: 'g', type: 'float32' },
    { name: 'b', type: 'float32' },
    { name: 'a', type: 'float32' },
  ];
  validateMessageStructure(spec, expectedFields);
  console.log('Message structure is valid');
});
```

### Code Generation Helper

```javascript
function generateFieldAccessors(spec) {
  const accessors = [];

  for (const field of spec.fields) {
    const getter = `get${field.name.charAt(0).toUpperCase() + field.name.slice(1)}() { return this.${field.name}; }`;
    const setter = `set${field.name.charAt(0).toUpperCase() + field.name.slice(1)}(value) { this.${field.name} = value; }`;
    accessors.push(getter, setter);
  }

  return accessors.join('\n');
}

// Generate accessors for a message
parser.parseMessageFile('std_msgs', colorRGBAPath).then((spec) => {
  const accessors = generateFieldAccessors(spec);
  console.log('Generated accessors:\n', accessors);
});
```

## Use Cases for ROSIDL Parsing

### Development Tools

- **IDE Integration**: Provide autocomplete and syntax highlighting
- **Interface Browsers**: Build tools to explore available interfaces
- **Documentation Generators**: Automatically generate interface documentation
- **Type Checkers**: Validate message usage in code

### Runtime Applications

- **Dynamic Message Handling**: Process messages with unknown structures
- **Bridge Implementations**: Convert between different message formats
- **Protocol Analyzers**: Inspect and analyze ROS 2 communication
- **Testing Frameworks**: Generate test data based on interface definitions

### Code Generation

- **Binding Generators**: Create language bindings for different platforms
- **Serialization Code**: Generate efficient serialization/deserialization
- **Mock Generators**: Create mock implementations for testing
- **Documentation**: Automatically generate API documentation

## Troubleshooting

### Common Issues

1. **AMENT_PREFIX_PATH Not Set**:

   ```
   Error: Cannot read property 'AMENT_PREFIX_PATH' of undefined
   ```

   Solution: Source your ROS 2 setup file:

   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **Package Not Found**:

   ```
   Error: ENOENT: no such file or directory
   ```

   Solution: Verify package installation:

   ```bash
   ros2 pkg list | grep std_msgs
   ```

3. **Permission Issues**:

   ```
   Error: EACCES: permission denied
   ```

   Solution: Check file permissions and ROS 2 installation

4. **Parsing Errors**:
   ```
   Error: Failed to parse interface file
   ```
   Solution: Verify interface file syntax and structure

### Debugging Tips

- Verify `AMENT_PREFIX_PATH` contains your ROS 2 installation
- Check that target packages are properly installed
- Use absolute paths for debugging file location issues
- Test with known-good interface files first
- Check ROS 2 installation completeness

## Notes

- ROSIDL parsing requires a properly configured ROS 2 environment
- Examples use standard ROS 2 packages that should be available in most installations
- Parser results provide complete interface structure information
- Field properties include array information and type details
- The parser handles both primitive and complex types
- Service and action parsing provide separate access to different interface sections
- Results can be used for code generation, validation, and dynamic message handling
