# ROS 2 Topics Examples

This directory contains examples demonstrating ROS 2 topic-based communication using rclnodejs. These examples showcase publishers, subscribers, and various messaging patterns available in ROS 2.

## Overview

ROS 2 topics are a fundamental communication pattern that allows nodes to exchange messages in a publish-subscribe manner. Publishers send messages to topics, and subscribers receive messages from topics they're interested in.

## Publisher Examples

The `publisher/` directory contains examples of nodes that publish messages to topics:

### 1. Basic Publisher (`publisher-example.js`)

**Purpose**: Demonstrates basic string message publishing.

- **Message Type**: `std_msgs/msg/String`
- **Topic**: `topic`
- **Functionality**: Publishes "Hello ROS" messages every second
- **Run Command**: `node publisher/publisher-example.js`

### 2. Content Filter Publisher (`publisher-content-filter-example.js`)

**Purpose**: Publishes temperature data for content filtering demonstrations.

- **Message Type**: `sensor_msgs/msg/Temperature`
- **Topic**: `temperature`
- **Functionality**: Publishes random temperature values (0-100°C) every 100ms with header information
- **Run Command**: `node publisher/publisher-content-filter-example.js`
- **Pair**: Works with `subscription-content-filter-example.js`

### 3. Message Publisher (`publisher-message-example.js`)

**Purpose**: Demonstrates publishing complex structured messages.

- **Message Type**: `sensor_msgs/msg/JointState`
- **Topic**: `JointState`
- **Functionality**: Publishes joint state information with header, names, positions, velocities, and efforts
- **Run Command**: `node publisher/publisher-message-example.js`
- **Pair**: Works with `subscription-message-example.js`

### 4. MultiArray Publisher (`publisher-multiarray-example.js`)

**Purpose**: Shows how to publish multi-dimensional array data.

- **Message Type**: `std_msgs/msg/Int32MultiArray`
- **Topic**: `Int32MultiArray`
- **Functionality**: Publishes 3D array data (2×3×3) with proper layout information
- **Run Command**: `node publisher/publisher-multiarray-example.js`
- **Pair**: Works with `subscription-multiarray-example.js`

### 5. QoS Publisher (`publisher-qos-example.js`)

**Purpose**: Demonstrates Quality of Service (QoS) configuration for publishers.

- **Message Type**: `std_msgs/msg/String`
- **Topic**: `topic`
- **Functionality**: Publishes messages with custom QoS settings (system default policies)
- **Run Command**: `node publisher/publisher-qos-example.js`
- **Pair**: Works with `subscription-qos-example.js`

### 6. Raw Message Publisher (`publisher-raw-message.js`)

**Purpose**: Shows how to publish raw binary data.

- **Message Type**: `test_msgs/msg/BasicTypes`
- **Topic**: `chatter`
- **Functionality**: Publishes raw Buffer data ("Hello ROS World")
- **Run Command**: `node publisher/publisher-raw-message.js`
- **Pair**: Works with `subscription-raw-message.js`

### 7. Publisher Validation (`publisher-validation-example.js`)

**Purpose**: Demonstrates message validation features for publishers.

- **Message Type**: `std_msgs/msg/String`, `geometry_msgs/msg/Twist`
- **Topics**: Various validation test topics
- **Functionality**:
  - Schema introspection with `getMessageSchema()`, `getFieldNames()`, `getFieldType()`
  - Publisher-level validation with `validateMessages: true` option
  - Per-publish validation override with `{ validate: true/false }`
  - Strict mode validation for unknown fields
  - Nested message validation (Twist with Vector3)
  - Reusable validators with `createMessageValidator()`
  - Error handling with `MessageValidationError`
- **Features**:
  - Catch invalid messages before publishing
  - Dynamic validation toggle with `willValidateMessage` property
  - Detailed error reports with field-level issues
- **Run Command**: `node publisher/publisher-validation-example.js`
- **Note**: Standalone example - no subscriber required

## Subscriber Examples

The `subscriber/` directory contains examples of nodes that subscribe to topics:

### 1. Basic Subscriber (`subscription-example.js`)

**Purpose**: Demonstrates basic message subscription.

- **Message Type**: `std_msgs/msg/String`
- **Topic**: `topic`
- **Functionality**: Receives and logs string messages
- **Run Command**: `node subscriber/subscription-example.js`
- **Pair**: Works with `publisher-example.js`

### 2. Content Filter Subscriber (`subscription-content-filter-example.js`)

**Purpose**: Demonstrates content filtering to receive only relevant messages.

- **Message Type**: `sensor_msgs/msg/Temperature`
- **Topic**: `temperature`
- **Functionality**: Only receives temperature messages above 50°C using content filters
- **Features**: ROS 2 Humble+ content filtering with expression `temperature > %0`
- **Run Command**: `node subscriber/subscription-content-filter-example.js`
- **Pair**: Works with `publisher-content-filter-example.js`

### 3. Message Subscriber (`subscription-message-example.js`)

**Purpose**: Receives complex structured messages.

- **Message Type**: `sensor_msgs/msg/JointState`
- **Topic**: `JointState`
- **Functionality**: Receives and logs joint state information
- **Run Command**: `node subscriber/subscription-message-example.js`
- **Pair**: Works with `publisher-message-example.js`

### 4. MultiArray Subscriber (`subscription-multiarray-example.js`)

**Purpose**: Demonstrates receiving and parsing multi-dimensional arrays.

- **Message Type**: `std_msgs/msg/Int32MultiArray`
- **Topic**: `Int32MultiArray`
- **Functionality**: Receives 3D arrays and iterates through all elements with proper indexing
- **Features**: Shows how to parse layout information and access array elements
- **Run Command**: `node subscriber/subscription-multiarray-example.js`
- **Pair**: Works with `publisher-multiarray-example.js`

### 5. QoS Subscriber (`subscription-qos-example.js`)

**Purpose**: Demonstrates QoS configuration for subscribers.

- **Message Type**: `std_msgs/msg/String`
- **Topic**: `topic`
- **Functionality**: Receives messages with system default QoS profile
- **Run Command**: `node subscriber/subscription-qos-example.js`
- **Pair**: Works with `publisher-qos-example.js`

### 6. Raw Message Subscriber (`subscription-raw-message.js`)

**Purpose**: Shows how to receive raw binary data.

- **Message Type**: `test_msgs/msg/BasicTypes`
- **Topic**: `chatter`
- **Functionality**: Receives raw Buffer data and converts to UTF-8 string
- **Features**: Uses `{ isRaw: true }` option
- **Run Command**: `node subscriber/subscription-raw-message.js`
- **Pair**: Works with `publisher-raw-message.js`

### 7. Service Event Subscriber (`subscription-service-event-example.js`)

**Purpose**: Demonstrates subscribing to service events.

- **Message Type**: `example_interfaces/srv/AddTwoInts_Event`
- **Topic**: `/add_two_ints/_service_event`
- **Functionality**: Monitors service call events for the AddTwoInts service
- **Features**: ROS 2 service introspection capabilities
- **Run Command**: `node subscriber/subscription-service-event-example.js`

### 8. Serialization Modes Subscriber (`subscription-serialization-modes-example.js`)

**Purpose**: Demonstrates different serialization modes for message handling.

- **Message Type**: `sensor_msgs/msg/LaserScan`
- **Topic**: `scan`
- **Functionality**: Shows how 'default', 'plain', and 'json' modes affect message serialization
- **Features**: Message serialization control for web applications and JSON compatibility
- **Run Command**: `node subscriber/subscription-serialization-modes-example.js`

### 9. JSON Utilities Subscriber (`subscription-json-utilities-example.js`)

**Purpose**: Demonstrates manual message conversion utilities.

- **Message Type**: `sensor_msgs/msg/LaserScan`
- **Topic**: `scan`
- **Functionality**: Shows how to use toJSONSafe and toJSONString utilities for manual conversion
- **Features**: Manual conversion of TypedArrays, BigInt, and special values for JSON serialization
- **Run Command**: `node subscriber/subscription-json-utilities-example.js`

## Validator Example

The `validator/` directory contains validation utilities:

### Validator (`validator-example.js`)

**Purpose**: Demonstrates ROS 2 name validation functions.

- **Functionality**: Validates topic names, node names, namespaces, and full topic names
- **Features**: Uses rclnodejs validator utilities
- **Run Command**: `node validator/validator-example.js`

## Paired Examples

Several examples work together to demonstrate complete communication:

| Publisher                             | Subscriber                               | Description                     |
| ------------------------------------- | ---------------------------------------- | ------------------------------- |
| `publisher-example.js`                | `subscription-example.js`                | Basic string messaging          |
| `publisher-content-filter-example.js` | `subscription-content-filter-example.js` | Temperature data with filtering |
| `publisher-message-example.js`        | `subscription-message-example.js`        | Complex structured messages     |
| `publisher-multiarray-example.js`     | `subscription-multiarray-example.js`     | Multi-dimensional array data    |
| `publisher-qos-example.js`            | `subscription-qos-example.js`            | QoS configuration               |
| `publisher-raw-message.js`            | `subscription-raw-message.js`            | Raw binary data                 |

## How to Run Examples

1. **Prerequisites**: Ensure ROS 2 is installed and sourced
2. **Navigate**: Change to the example/topics directory
3. **Run Publisher**: Start the publisher in one terminal
   ```bash
   node publisher/publisher-example.js
   ```
4. **Run Subscriber**: Start the corresponding subscriber in another terminal
   ```bash
   node subscriber/subscription-example.js
   ```

## Key Concepts Demonstrated

- **Basic Pub/Sub**: Simple message exchange patterns
- **Message Types**: Various ROS 2 message types (String, Temperature, JointState, MultiArray)
- **QoS Configuration**: Quality of Service settings for reliable communication
- **Content Filtering**: Selective message reception based on content (ROS 2 Humble+)
- **Raw Messages**: Binary data transmission
- **Service Events**: Monitoring service interactions
- **Multi-dimensional Arrays**: Complex data structures with layout information
- **Message Serialization**: TypedArray handling and JSON-safe conversion for web applications
- **Name Validation**: Topic names, node names, and namespace validation utilities
- **Message Validation**: Schema introspection and pre-publish message validation with detailed error reporting

## Notes

- All examples use the standard rclnodejs initialization pattern
- Most examples run continuously until terminated (Ctrl+C)
- Content filtering requires ROS 2 Humble or later
- Raw message examples require matching message types between publisher and subscriber
- Service event monitoring works with any service but requires the service to be active
