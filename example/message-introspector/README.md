# ROS 2 MessageIntrospector Example

This directory contains an example demonstrating the `MessageIntrospector` class for inspecting ROS 2 message structure.

## Overview

The `MessageIntrospector` class provides a simple way to understand the structure of ROS 2 messages without directly using `loader.loadInterface`. It's useful for debugging, generating documentation, and building dynamic UIs based on message structure.

## MessageIntrospector Example (`message-introspector-example.mjs`)

**Purpose**: Demonstrates how to inspect message structure, fields, and default values.

### Run Command

```bash
node message-introspector-example.mjs
```

### Expected Output

```
Twist fields: [ 'linear', 'angular' ]
Twist defaults: { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } }
String fields: [ 'data' ]
String defaults: { data: '' }
JointState fields: [ 'header', 'name', 'position', 'velocity', 'effort' ]
Twist schema msgName: Twist
```

## API

```javascript
const Twist = new rclnodejs.MessageIntrospector('geometry_msgs/msg/Twist');

Twist.typeName; // 'geometry_msgs/msg/Twist'
Twist.fields; // ['linear', 'angular']
Twist.defaults; // { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } }
Twist.schema; // ROSMessageDef object
Twist.typeClass; // The underlying constructor
```

## Notes

- Works with any message type including custom packages
- Default values are cached for performance after the first access
- The `defaults` getter returns a new deep copy each time to prevent mutation
- Service request/response types can also be inspected (e.g., `'my_pkg/srv/MyService_Request'`)
