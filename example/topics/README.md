# ROS 2 Topics Examples

This directory contains examples demonstrating ROS 2 topic-based communication using rclnodejs. These examples showcase publishers, subscribers, and various messaging patterns available in ROS 2.

## Overview

ROS 2 topics are a fundamental communication pattern that allows nodes to exchange messages in a publish-subscribe manner. Publishers send messages to topics, and subscribers receive messages from topics they're interested in.

## Publisher Examples

The `publisher/` directory contains examples of nodes that publish messages to topics:

### 1. Basic Publisher (`publisher-example.cjs`)

**Purpose**: Demonstrates basic string message publishing.

- **Message Type**: `std_msgs/msg/String`
- **Topic**: `topic`
- **Functionality**: Publishes "Hello ROS" messages every second
- **Run Command**: `node publisher/publisher-example.cjs`

### 2. Content Filter Publisher (`publisher-content-filter-example.cjs`)

**Purpose**: Publishes temperature data for content filtering demonstrations.

- **Message Type**: `sensor_msgs/msg/Temperature`
- **Topic**: `temperature`
- **Functionality**: Publishes random temperature values (0-100°C) every 100ms with header information
- **Run Command**: `node publisher/publisher-content-filter-example.cjs`
- **Pair**: Works with `subscription-content-filter-example.cjs`

### 3. Message Publisher (`publisher-message-example.cjs`)

**Purpose**: Demonstrates publishing complex structured messages.

- **Message Type**: `sensor_msgs/msg/JointState`
- **Topic**: `JointState`
- **Functionality**: Publishes joint state information with header, names, positions, velocities, and efforts
- **Run Command**: `node publisher/publisher-message-example.cjs`
- **Pair**: Works with `subscription-message-example.cjs`

### 4. MultiArray Publisher (`publisher-multiarray-example.cjs`)

**Purpose**: Shows how to publish multi-dimensional array data.

- **Message Type**: `std_msgs/msg/Int32MultiArray`
- **Topic**: `Int32MultiArray`
- **Functionality**: Publishes 3D array data (2×3×3) with proper layout information
- **Run Command**: `node publisher/publisher-multiarray-example.cjs`
- **Pair**: Works with `subscription-multiarray-example.cjs`

### 5. QoS Publisher (`publisher-qos-example.cjs`)

**Purpose**: Demonstrates Quality of Service (QoS) configuration for publishers.

- **Message Type**: `std_msgs/msg/String`
- **Topic**: `topic`
- **Functionality**: Publishes messages with custom QoS settings (system default policies)
- **Run Command**: `node publisher/publisher-qos-example.cjs`
- **Pair**: Works with `subscription-qos-example.cjs`

### 6. Raw Message Publisher (`publisher-raw-message.cjs`)

**Purpose**: Shows how to publish raw binary data.

- **Message Type**: `test_msgs/msg/BasicTypes`
- **Topic**: `chatter`
- **Functionality**: Publishes raw Buffer data ("Hello ROS World")
- **Run Command**: `node publisher/publisher-raw-message.cjs`
- **Pair**: Works with `subscription-raw-message.cjs`

### 7. Publisher Validation (`publisher-validation-example.cjs`)

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
- **Run Command**: `node publisher/publisher-validation-example.cjs`
- **Note**: Standalone example - no subscriber required

## Subscriber Examples

The `subscriber/` directory contains examples of nodes that subscribe to topics:

### 1. Basic Subscriber (`subscription-example.cjs`)

**Purpose**: Demonstrates basic message subscription.

- **Message Type**: `std_msgs/msg/String`
- **Topic**: `topic`
- **Functionality**: Receives and logs string messages
- **Run Command**: `node subscriber/subscription-example.cjs`
- **Pair**: Works with `publisher-example.cjs`

### 2. Content Filter Subscriber (`subscription-content-filter-example.cjs`)

**Purpose**: Demonstrates content filtering to receive only relevant messages.

- **Message Type**: `sensor_msgs/msg/Temperature`
- **Topic**: `temperature`
- **Functionality**: Only receives temperature messages above 50°C using content filters
- **Features**: ROS 2 Humble+ content filtering with expression `temperature > %0`
- **Run Command**: `node subscriber/subscription-content-filter-example.cjs`
- **Pair**: Works with `publisher-content-filter-example.cjs`

### 3. Message Subscriber (`subscription-message-example.cjs`)

**Purpose**: Receives complex structured messages.

- **Message Type**: `sensor_msgs/msg/JointState`
- **Topic**: `JointState`
- **Functionality**: Receives and logs joint state information
- **Run Command**: `node subscriber/subscription-message-example.cjs`
- **Pair**: Works with `publisher-message-example.cjs`

### 4. MultiArray Subscriber (`subscription-multiarray-example.cjs`)

**Purpose**: Demonstrates receiving and parsing multi-dimensional arrays.

- **Message Type**: `std_msgs/msg/Int32MultiArray`
- **Topic**: `Int32MultiArray`
- **Functionality**: Receives 3D arrays and iterates through all elements with proper indexing
- **Features**: Shows how to parse layout information and access array elements
- **Run Command**: `node subscriber/subscription-multiarray-example.cjs`
- **Pair**: Works with `publisher-multiarray-example.cjs`

### 5. QoS Subscriber (`subscription-qos-example.cjs`)

**Purpose**: Demonstrates QoS configuration for subscribers.

- **Message Type**: `std_msgs/msg/String`
- **Topic**: `topic`
- **Functionality**: Receives messages with system default QoS profile
- **Run Command**: `node subscriber/subscription-qos-example.cjs`
- **Pair**: Works with `publisher-qos-example.cjs`

### 6. Raw Message Subscriber (`subscription-raw-message.cjs`)

**Purpose**: Shows how to receive raw binary data.

- **Message Type**: `test_msgs/msg/BasicTypes`
- **Topic**: `chatter`
- **Functionality**: Receives raw Buffer data and converts to UTF-8 string
- **Features**: Uses `{ isRaw: true }` option
- **Run Command**: `node subscriber/subscription-raw-message.cjs`
- **Pair**: Works with `publisher-raw-message.cjs`

### 7. Service Event Subscriber (`subscription-service-event-example.cjs`)

**Purpose**: Demonstrates subscribing to service events.

- **Message Type**: `example_interfaces/srv/AddTwoInts_Event`
- **Topic**: `/add_two_ints/_service_event`
- **Functionality**: Monitors service call events for the AddTwoInts service
- **Features**: ROS 2 service introspection capabilities
- **Run Command**: `node subscriber/subscription-service-event-example.cjs`

### 8. Serialization Modes Subscriber (`subscription-serialization-modes-example.cjs`)

**Purpose**: Demonstrates different serialization modes for message handling.

- **Message Type**: `sensor_msgs/msg/LaserScan`
- **Topic**: `scan`
- **Functionality**: Shows how 'default', 'plain', and 'json' modes affect message serialization
- **Features**: Message serialization control for web applications and JSON compatibility
- **Run Command**: `node subscriber/subscription-serialization-modes-example.cjs`

### 9. JSON Utilities Subscriber (`subscription-json-utilities-example.cjs`)

**Purpose**: Demonstrates manual message conversion utilities.

- **Message Type**: `sensor_msgs/msg/LaserScan`
- **Topic**: `scan`
- **Functionality**: Shows how to use toJSONSafe and toJSONString utilities for manual conversion
- **Features**: Manual conversion of TypedArrays, BigInt, and special values for JSON serialization
- **Run Command**: `node subscriber/subscription-json-utilities-example.cjs`

### 10. Observable Subscriber (`subscription-observable-example.cjs`)

**Purpose**: Demonstrates RxJS Observable subscriptions for reactive programming.

- **Message Type**: `std_msgs/msg/String`
- **Topic**: `topic`
- **Functionality**: Shows how to use `createObservableSubscription()` with RxJS operators
- **Features**:
  - Throttling with `throttleTime()` for rate limiting
  - Message transformation with `map()`
  - Content filtering with `filter()`
  - Batching with `bufferCount()`
- **Run Command**: `node subscriber/subscription-observable-example.cjs`
- **Pair**: Works with `publisher-example.cjs`

## Validator Example

The `validator/` directory contains validation utilities:

### Validator (`validator-example.cjs`)

**Purpose**: Demonstrates ROS 2 name validation functions.

- **Functionality**: Validates topic names, node names, namespaces, and full topic names
- **Features**: Uses rclnodejs validator utilities
- **Run Command**: `node validator/validator-example.cjs`

## Paired Examples

Several examples work together to demonstrate complete communication:

| Publisher                             | Subscriber                               | Description                     |
| ------------------------------------- | ---------------------------------------- | ------------------------------- |
| `publisher-example.cjs`                | `subscription-example.cjs`                | Basic string messaging          |
| `publisher-content-filter-example.cjs` | `subscription-content-filter-example.cjs` | Temperature data with filtering |
| `publisher-message-example.cjs`        | `subscription-message-example.cjs`        | Complex structured messages     |
| `publisher-multiarray-example.cjs`     | `subscription-multiarray-example.cjs`     | Multi-dimensional array data    |
| `publisher-qos-example.cjs`            | `subscription-qos-example.cjs`            | QoS configuration               |
| `publisher-raw-message.cjs`            | `subscription-raw-message.cjs`            | Raw binary data                 |
| `publisher-example.cjs`                | `subscription-observable-example.cjs`     | RxJS Observable subscription    |

## How to Run Examples

1. **Prerequisites**: Ensure ROS 2 is installed and sourced
2. **Navigate**: Change to the example/topics directory
3. **Run Publisher**: Start the publisher in one terminal
   ```bash
   node publisher/publisher-example.cjs
   ```
4. **Run Subscriber**: Start the corresponding subscriber in another terminal
   ```bash
   node subscriber/subscription-example.cjs
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
- **Observable Subscriptions**: RxJS-based reactive programming with operators for throttling, filtering, and combining message streams

## Notes

- All examples use the standard rclnodejs initialization pattern
- Most examples run continuously until terminated (Ctrl+C)
- Content filtering requires ROS 2 Humble or later
- Raw message examples require matching message types between publisher and subscriber
- Service event monitoring works with any service but requires the service to be active
