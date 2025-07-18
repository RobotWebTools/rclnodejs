# ROS 2 Parameters Examples

This directory contains examples demonstrating ROS 2 parameter handling using rclnodejs. Parameters provide a way to configure node behavior at runtime and can be modified dynamically during execution.

## Overview

ROS 2 parameters are configuration values that can be set on nodes to modify their behavior. Parameters are:

- **Typed**: Each parameter has a specific type (string, int, double, bool, etc.)
- **Declarable**: Must be declared before use with optional descriptors
- **Dynamic**: Can be changed at runtime through services or command line
- **Discoverable**: Can be listed and inspected by other nodes and tools

Parameters are ideal for:

- Configuration settings (rates, thresholds, file paths)
- Behavior modification without code changes
- Runtime tuning and debugging
- Node customization for different environments

## Parameter Examples

### 1. Parameter Declaration (`parameter-declaration-example.js`)

**Purpose**: Demonstrates how to declare and use parameters in a ROS 2 node.

- **Functionality**:
  - Creates a node named `my_node`
  - Declares a string parameter `param1` with default value "hello world"
  - Uses `ParameterDescriptor` to define parameter metadata
  - Checks parameter existence and retrieves its value
  - Displays parameter details and descriptor information
- **Features**:
  - Parameter declaration with `declareParameter()`
  - Parameter existence checking with `hasParameter()`
  - Parameter value retrieval with `getParameter()`
  - Parameter descriptor access with `getParameterDescriptor()`
- **Parameter Details**:
  - **Name**: `param1`
  - **Type**: `PARAMETER_STRING`
  - **Default Value**: `"hello world"`
- **Run Command**: `node parameter-declaration-example.js`

### 2. Parameter Override (`parameter-override-example.js`)

**Purpose**: Shows how to override parameter values using command-line arguments.

- **Functionality**:
  - Creates a node named `my_node`
  - Declares a string parameter `param1` with default value "hello world"
  - Demonstrates command-line parameter override using ROS args
  - Shows how the parameter value is changed from default to override value
  - Displays the overridden parameter value and descriptor
- **Features**:
  - Command-line parameter override with `--ros-args -p`
  - ROS argument parsing integration
  - Parameter initialization with custom argv
  - Comparison between default and overridden values
- **Parameter Details**:
  - **Name**: `param1`
  - **Type**: `PARAMETER_STRING`
  - **Default Value**: `"hello world"`
  - **Override Value**: `"hello ros2"` (via command line)
- **Run Command**: `node parameter-override-example.js`

## How to Run the Examples

### Prerequisites

1. Ensure ROS 2 is installed and sourced
2. Navigate to the `example/parameter` directory

### Running Parameter Declaration Example

```bash
cd example/parameter
node parameter-declaration-example.js
```

**Expected Output**:

```
Declared parameter: param1
Parameter details:  Parameter {
  name: 'param1',
  type: 4,
  value: 'hello world'
}
ParameterDescriptor {
  name: 'param1',
  type: 4,
  description: '',
  additional_constraints: '',
  read_only: false,
  dynamic_typing: false,
  floating_point_range: [],
  integer_range: []
}
```

### Running Parameter Override Example

```bash
cd example/parameter
node parameter-override-example.js
```

**Expected Output**:

```
Declared parameter: param1
Parameter overridden:  Parameter {
  name: 'param1',
  type: 4,
  value: 'hello ros2'
}
ParameterDescriptor {
  name: 'param1',
  type: 4,
  description: '',
  additional_constraints: '',
  read_only: false,
  dynamic_typing: false,
  floating_point_range: [],
  integer_range: []
}
```

Notice how the value changed from "hello world" to "hello ros2" due to the command-line override.

## Using ROS 2 Parameter Tools

You can interact with these examples using standard ROS 2 parameter tools:

### Listing Parameters

While either example is running, you can list parameters:

```bash
ros2 param list
```

Expected output:

```
/my_node:
  param1
  use_sim_time
```

### Getting Parameter Values

```bash
ros2 param get /my_node param1
```

Expected output:

```
String value is: hello world
```

(or "hello ros2" for the override example)

### Setting Parameter Values

```bash
ros2 param set /my_node param1 "new value"
```

### Describing Parameters

```bash
ros2 param describe /my_node param1
```

## Key Concepts Demonstrated

### Parameter Declaration

- **Parameter Objects**: Creating `Parameter` instances with name, type, and value
- **Parameter Descriptors**: Using `ParameterDescriptor` for metadata
- **Declaration Process**: Registering parameters with the node using `declareParameter()`
- **Type Safety**: Ensuring parameter types match expected values

### Parameter Types

The examples demonstrate `PARAMETER_STRING`, but ROS 2 supports various types:

- `PARAMETER_BOOL`: Boolean values
- `PARAMETER_INTEGER`: 64-bit signed integers
- `PARAMETER_DOUBLE`: Double-precision floating point
- `PARAMETER_STRING`: UTF-8 encoded strings
- `PARAMETER_BYTE_ARRAY`: Raw byte arrays
- `PARAMETER_BOOL_ARRAY`: Arrays of booleans
- `PARAMETER_INTEGER_ARRAY`: Arrays of integers
- `PARAMETER_DOUBLE_ARRAY`: Arrays of doubles
- `PARAMETER_STRING_ARRAY`: Arrays of strings

### Parameter Management

- **Existence Checking**: Using `hasParameter()` to verify parameter presence
- **Value Retrieval**: Getting current parameter values with `getParameter()`
- **Descriptor Access**: Retrieving parameter metadata with `getParameterDescriptor()`
- **Runtime Override**: Modifying parameters via command line or services

### Command-Line Integration

- **ROS Args**: Using `--ros-args -p` syntax for parameter overrides
- **Node-Specific Parameters**: Targeting specific nodes with `node_name:param_name:=value`
- **Initialization Integration**: Passing argv to `rclnodejs.init()` for parameter processing

## Advanced Parameter Usage

### Parameter Constraints

ParameterDescriptor can include constraints:

```javascript
const descriptor = new ParameterDescriptor(
  'my_param',
  ParameterType.PARAMETER_INTEGER
);
descriptor.description = 'A sample integer parameter';
descriptor.additional_constraints = 'Must be positive';
descriptor.read_only = false;
descriptor.integer_range = [{ from_value: 0, to_value: 100, step: 1 }];
```

### Parameter Callbacks

You can set up callbacks to respond to parameter changes:

```javascript
node.setParameterCallback((parameters) => {
  for (const param of parameters) {
    console.log(`Parameter ${param.name} changed to ${param.value}`);
  }
});
```

### Multiple Parameter Types

```javascript
// String parameter
const stringParam = new Parameter(
  'config_file',
  ParameterType.PARAMETER_STRING,
  '/path/to/config'
);

// Integer parameter
const intParam = new Parameter(
  'max_items',
  ParameterType.PARAMETER_INTEGER,
  100
);

// Boolean parameter
const boolParam = new Parameter(
  'debug_mode',
  ParameterType.PARAMETER_BOOL,
  false
);

// Double parameter
const doubleParam = new Parameter(
  'update_rate',
  ParameterType.PARAMETER_DOUBLE,
  10.0
);
```

## Troubleshooting

### Common Issues

1. **Parameter Not Found**:

   - Ensure parameter is declared before accessing
   - Check parameter name spelling
   - Verify node has been properly initialized

2. **Type Mismatch**:

   - Ensure parameter type matches declaration
   - Check ParameterType constants are correct
   - Verify value type matches parameter type

3. **Override Not Working**:

   - Check command-line syntax: `--ros-args -p node_name:param_name:=value`
   - Ensure node name matches exactly
   - Verify rclnodejs.init() is called with argv

4. **Permission Issues**:
   - Check if parameter is marked as read_only
   - Verify parameter constraints are satisfied

### Debugging Tips

- Use `ros2 param list` to see all available parameters
- Use `ros2 param describe <node_name> <param_name>` to check parameter details
- Use `ros2 param get <node_name> <param_name>` to verify current values
- Check console output for parameter declaration confirmations
- Verify node names match between declaration and command-line usage

## Notes

- Parameters must be declared before they can be used or accessed
- Parameter types are enforced and cannot be changed after declaration
- Command-line overrides are processed during `rclnodejs.init()`
- Parameter descriptors provide metadata but are optional
- The `use_sim_time` parameter is automatically added to all nodes
- Parameter names are case-sensitive and must be valid ROS names
- Parameters are node-specific and accessed using the full node path
