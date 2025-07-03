# ROS2 IDL to Interface Converter

This Python tool converts ROS2 `.idl` files to corresponding `.msg`, `.srv`, and `.action` files.

## Features

- **Complete IDL Parsing**: Parses ROS2 IDL syntax including modules, structs, sequences, and arrays
- **Type Mapping**: Automatically maps IDL types to ROS2 types (e.g., `double` → `float64`, `sequence<T>` → `T[]`)
- **Typedef Support**: Handles both simple and array typedefs for complex type definitions
- **Constants and Default Values**: Supports constant definitions and field default values with `@default` annotations
- **Comment Preservation**: Extracts and preserves comments from `@verbatim` blocks
- **Key Annotation Detection**: Automatically skips IDL files with `@key` annotations (not supported in ROS2)
- **Multi-Interface Support**: Handles messages, services, and actions in a single IDL file
- **Namespace Support**: Properly handles namespaced types (e.g., `std_msgs::msg::Header` → `std_msgs/Header`)
- **Command Line Interface**: Easy to use with command line arguments
- **Verbose Output**: Optional detailed output showing parsed structures and generated files

## Usage

### Basic Usage

```bash
python3 idl_convertor.py <idl_file>
```

### With Options

```bash
python3 idl_convertor.py <idl_file> [options]
```

### Options

- `-o, --output DIR`: Output directory name for generated files (default: `ros_interfaces`)
- `-r, --root PATH`: Root path where the generated files will be located (default: current directory)
- `-p, --package NAME`: Package name to use for generated files (overrides package name from IDL)
- `-v, --verbose`: Enable verbose output showing parsed structures and file contents
- `-h, --help`: Show help message

### Advanced Examples

#### Custom Output Directory

```bash
python3 idl_convertor.py JointState.idl -o my_interfaces
```

#### Custom Root Path

```bash
python3 idl_convertor.py SetCameraInfo.idl -r /path/to/workspace -o sensor_msgs
# Generates files in: /path/to/workspace/sensor_msgs/srv/SetCameraInfo.srv
```

#### Custom Package Name

```bash
python3 idl_convertor.py JointState.idl -p my_package_name
# Overrides the package name from the IDL file
```

#### Combined Options

```bash
python3 idl_convertor.py SetCameraInfo.idl -r ~/ros2_ws/src -o sensor_msgs -p sensor_msgs -v
# Generates: ~/ros2_ws/src/sensor_msgs/srv/SetCameraInfo.srv with package name "sensor_msgs"
```

## Examples

### 1. Convert a Message IDL

Input file `JointState.idl`:

```idl
#include "std_msgs/msg/Header.idl"

module sensor_msgs {
  module msg {
    struct JointState {
      std_msgs::msg::Header header;
      sequence<string> name;
      sequence<double> position;
      sequence<double> velocity;
      sequence<double> effort;
    };
  };
};
```

Output `JointState.msg`:

```
# JointState.msg
# Generated from IDL file

std_msgs/Header header
string[] name
float64[] position
float64[] velocity
float64[] effort
```

### 2. Convert a Service IDL

Input file `SetCameraInfo.idl`:

```idl
#include "sensor_msgs/msg/CameraInfo.idl"

module sensor_msgs {
  module srv {
    struct SetCameraInfo_Request {
      sensor_msgs::msg::CameraInfo camera_info;
    };
    struct SetCameraInfo_Response {
      boolean success;
      string status_message;
    };
  };
};
```

Output `SetCameraInfo.srv`:

```
# SetCameraInfo.srv
# Generated from IDL file

# Request
sensor_msgs/CameraInfo camera_info
---
# Response
bool success
string status_message
```

### 3. Convert an Action IDL

Input file `Fibonacci.idl`:

```idl
module example_interfaces {
  module action {
    struct FibonacciGoal {
      int32 order;
    };

    struct FibonacciResult {
      sequence<int32> sequence;
    };

    struct FibonacciFeedback {
      sequence<int32> partial_sequence;
    };
  };
};
```

Generates separate message files for goal, result, and feedback components.

## Type Mappings

### Basic Type Mappings

| IDL Type         | ROS2 Type  |
| ---------------- | ---------- |
| `boolean`        | `bool`     |
| `octet`          | `uint8`    |
| `int8`           | `int8`     |
| `uint8`          | `uint8`    |
| `int16`          | `int16`    |
| `uint16`         | `uint16`   |
| `int32`          | `int32`    |
| `uint32`         | `uint32`   |
| `int64`          | `int64`    |
| `uint64`         | `uint64`   |
| `float`          | `float32`  |
| `double`         | `float64`  |
| `string`         | `string`   |
| `wstring`        | `wstring`  |
| `sequence<T>`    | `T[]`      |
| `T[N]`           | `T[N]`     |
| `pkg::msg::Type` | `pkg/Type` |

### Typedef Support

The tool supports both simple and array typedefs:

- **Simple typedef**: `typedef double MyDouble;` → Maps `MyDouble` to `float64`
- **Array typedef**: `typedef double MyArray[9];` → Maps `MyArray` to `float64[9]`
- **Namespaced typedef**: `typedef std_msgs::msg::Header HeaderType;` → Maps `HeaderType` to `std_msgs/Header`

## Output Structure

The tool creates the following directory structure:

```
<root>/<output>/
├── msg/           # Generated .msg files
├── srv/           # Generated .srv files
└── action/        # Generated .action files
```

### ROS2 Workspace Integration

For proper ROS2 workspace integration, you can use the parameters to match the expected structure:

```bash
# Generate files for a ROS2 package in a workspace
python3 idl_convertor.py MyMessage.idl \
  -r ~/ros2_ws/src \
  -o my_package_name \
  -p my_package_name

# This creates:
# ~/ros2_ws/src/my_package_name/msg/MyMessage.msg
# ~/ros2_ws/src/my_package_name/srv/MyService.srv
# ~/ros2_ws/src/my_package_name/action/MyAction.action
```

The generated files will be compatible with ROS2 build tools like `colcon build`.

## Important Notes

### DDS @key Annotation Handling

The tool automatically detects and skips IDL files that contain:

- Direct `@key` annotations (e.g., `@key string identifier;`)
- References to types that use `@key` annotations (e.g., `KeyedString`, `KeyedLong`)

This is because `@key` annotations are DDS-specific features that are not supported in ROS2 .msg files. When such files are encountered, the tool will print a warning and skip processing:

```
Warning: Skipping MyFile.idl - contains @key annotations which are not supported in ROS2 .msg files
```

or

```
Warning: Skipping MyFile.idl - references keyed types which are not supported in ROS2 .msg files
```

## Implementation Details

### Classes

- **`IdlParser`**: Parses IDL files and extracts interface definitions
- **`RosInterfaceGenerator`**: Generates ROS2 interface files from parsed data
- **`IdlField`**: Represents a field in an IDL structure (with support for comments and default values)
- **`IdlConstant`**: Represents a constant definition in an IDL structure
- **`IdlStructure`**: Represents an IDL structure (message, service part, etc.)
- **`IdlInterface`**: Represents a complete IDL interface definition

### Key Features

- **Robust Parsing**: Handles comments, nested modules, typedefs, and complex type definitions
- **Key Annotation Detection**: Automatically detects and skips files with `@key` annotations
- **Comment Preservation**: Extracts comments from `@verbatim` blocks and associates them with fields
- **Default Value Support**: Processes `@default` annotations and formats them for ROS2
- **Error Handling**: Graceful error handling with informative messages
- **Extensible**: Easy to extend for additional IDL features or output formats

## Testing

The tool has been tested with:

- ✅ Basic message types (JointState)
- ✅ Service definitions (SetCameraInfo) - generates proper .srv files
- ✅ Action definitions (Fibonacci) - generates proper .action files
- ✅ Array and sequence types
- ✅ Namespaced types
- ✅ Typedef declarations (simple and array types)
- ✅ Constants and default values with `@default` annotations
- ✅ Comment preservation from `@verbatim` blocks
- ✅ `@key` annotation detection and file skipping
- ✅ Command line interface with all options
- ✅ Request/Response combination for services
- ✅ Goal/Result/Feedback combination for actions
- ✅ Field order preservation from IDL to generated files

## Future Enhancements

- [ ] Support for nested structures and complex type inheritance
- [ ] Support for enums and unions
- [ ] Support for IDL annotations beyond `@verbatim`, `@default`, and `@key`
- [ ] Validation of generated files against ROS2 interface specifications
- [ ] Support for composition and inheritance patterns
- [ ] Batch processing of multiple IDL files
- [ ] Integration with ROS2 build tools (ament, colcon)

## Requirements

- Python 3.6+
- No external dependencies (uses only standard library)

This tool provides a robust solution for converting ROS2 IDL files to standard ROS2 interface formats, making it easier to work with interface definitions across different ROS2 tools and languages.
