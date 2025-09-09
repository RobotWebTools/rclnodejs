# Custom Message Test Package

This ROS2 package contains custom message definitions for testing rclnodejs runtime message generation capabilities.

## Messages

### Testing.msg

A test message that contains the position of a point in free space with additional data:

- `float64 x` - X coordinate
- `float64 y` - Y coordinate
- `float64 z` - Z coordinate
- `string data` - Additional string data

## Usage

This package is used by the rclnodejs test suite to verify that custom messages can be generated and used at runtime.

## Building

```bash
# From the test directory
cd custom_msg_test
colcon build
source install/setup.bash
```

## Testing

The message definitions in this package are used by `test-rosidl-message-generator.js` to verify runtime message generation functionality.
