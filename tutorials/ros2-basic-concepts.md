# ROS 2 Basic Concepts Tutorial

This tutorial introduces the fundamental communication patterns in ROS 2: **Topics** (publish/subscribe) and **Services** (request/response). These are essential building blocks for any ROS 2 application.

## Table of Contents

- [What are ROS 2 Communication Patterns?](#what-are-ros-2-communication-patterns)
- [Topics (Publish/Subscribe)](#topics-publishsubscribe)
- [Services (Request/Response)](#services-requestresponse)
- [When to Use Topics vs Services](#when-to-use-topics-vs-services)
- [Running the Examples](#running-the-examples)

## What are ROS 2 Communication Patterns?

ROS 2 provides three primary communication patterns:

- **📡 Topics** - Continuous data streams (pub/sub)
- **🔧 Services** - Remote procedure calls (request/response)
- **⚡ Actions** - Long-running tasks with feedback (covered in separate tutorial)

These patterns enable **distributed communication** between nodes in a robotics system, allowing for flexible, modular architectures.

## Topics (Publish/Subscribe)

### Concept Overview

**Topics** implement a **publish/subscribe** communication pattern where:

- **Publishers** produce data and send it to a named topic
- **Subscribers** consume data from the same named topic
- **Anonymous** - Subscribers don't know which publisher sent the data
- **Many-to-many** - Multiple publishers and subscribers per topic
- **Asynchronous** - Publishers don't wait for subscribers

Topics are ideal for **continuous data streams** like sensor readings, robot state, or camera images.

### Basic Publisher Example

```javascript
const rclnodejs = require('rclnodejs');

async function createPublisher() {
  await rclnodejs.init();
  const node = rclnodejs.createNode('publisher_example_node');

  // Create a publisher for String messages on 'topic'
  const publisher = node.createPublisher('std_msgs/msg/String', 'topic');

  let counter = 0;
  setInterval(() => {
    const message = `Hello ROS ${counter}`;
    console.log(`Publishing message: ${message}`);
    publisher.publish(message);
    counter++;
  }, 1000);

  rclnodejs.spin(node);
}

createPublisher().catch(console.error);
```

### Basic Subscriber Example

```javascript
const rclnodejs = require('rclnodejs');

async function createSubscriber() {
  await rclnodejs.init();
  const node = rclnodejs.createNode('subscriber_example_node');

  // Create a subscriber for String messages on 'topic'
  node.createSubscription('std_msgs/msg/String', 'topic', (msg) => {
    console.log(`Received message: ${typeof msg}`, msg);
  });

  rclnodejs.spin(node);
}

createSubscriber().catch(console.error);
```

### Topic Features

- **Strongly Typed** - Messages have well-defined types (e.g., `std_msgs/msg/String`)
- **Buffered** - Publishers can send data even if no subscribers exist
- **Discoverable** - Use `ros2 topic list` to see available topics
- **Quality of Service** - Configure reliability, durability, and latency

### Advanced Publisher with Custom Messages

```javascript
const rclnodejs = require('rclnodejs');

async function publishSensorData() {
  await rclnodejs.init();
  const node = rclnodejs.createNode('sensor_publisher');

  // Publisher for geometry messages
  const publisher = node.createPublisher('geometry_msgs/msg/Twist', 'cmd_vel');

  setInterval(() => {
    // Create a Twist message for robot velocity
    const twist = {
      linear: { x: 1.0, y: 0.0, z: 0.0 },
      angular: { x: 0.0, y: 0.0, z: 0.5 },
    };

    console.log('Publishing velocity command');
    publisher.publish(twist);
  }, 100); // 10 Hz

  rclnodejs.spin(node);
}

publishSensorData().catch(console.error);
```

## Services (Request/Response)

### Concept Overview

**Services** implement a **request/response** communication pattern where:

- **Service Server** provides a computation/service
- **Service Client** requests the service and waits for response
- **Synchronous** - Client waits for server response
- **One-to-one** - One server per service name, multiple clients allowed
- **Short-lived** - Services should return quickly

Services are ideal for **remote procedure calls**, configuration requests, or triggering specific actions.

### Basic Service Server Example

```javascript
const rclnodejs = require('rclnodejs');

async function createServiceServer() {
  await rclnodejs.init();
  const node = rclnodejs.createNode('service_example_node');

  // Create a service that adds two integers
  const service = node.createService(
    'example_interfaces/srv/AddTwoInts',
    'add_two_ints',
    (request, response) => {
      console.log(`Request: ${request.a} + ${request.b}`);

      // Compute the result
      const result = response.template;
      result.sum = request.a + request.b;

      console.log(`Sending response: ${typeof result}`, result);
      response.send(result);
    }
  );

  console.log('Service server ready');
  rclnodejs.spin(node);
}

createServiceServer().catch(console.error);
```

### Basic Service Client Example

```javascript
const rclnodejs = require('rclnodejs');

async function createServiceClient() {
  await rclnodejs.init();
  const node = rclnodejs.createNode('client_example_node');

  // Create a client for the add_two_ints service
  const client = node.createClient(
    'example_interfaces/srv/AddTwoInts',
    'add_two_ints'
  );

  // Wait for service to become available
  const serviceAvailable = await client.waitForService(5000);
  if (!serviceAvailable) {
    console.log('Service not available');
    rclnodejs.shutdown();
    return;
  }

  // Create request
  const request = {
    a: BigInt(10),
    b: BigInt(15),
  };

  console.log(`Calling service with: ${request.a} + ${request.b}`);

  // Send request with callback
  client.sendRequest(request, (response) => {
    console.log(`Result: ${typeof response}`, response);
    rclnodejs.shutdown();
  });

  rclnodejs.spin(node);
}

createServiceClient().catch(console.error);
```

### Service Features

- **Request/Response Structure** - Services define both request and response message types
- **Blocking** - Clients wait for server response
- **Error Handling** - Services can fail and return errors
- **Discoverability** - Use `ros2 service list` to see available services

### Practical Service Example: Robot Configuration

```javascript
const rclnodejs = require('rclnodejs');

class RobotConfigurationService {
  constructor() {
    this.robotConfig = {
      maxSpeed: 2.0,
      safetyEnabled: true,
      operationMode: 'autonomous',
    };
  }

  async start() {
    await rclnodejs.init();
    this.node = rclnodejs.createNode('robot_config_service');

    // Service to get robot configuration
    this.node.createService(
      'example_interfaces/srv/Trigger',
      'get_robot_config',
      (request, response) => {
        const result = response.template;
        result.success = true;
        result.message = JSON.stringify(this.robotConfig);
        response.send(result);
      }
    );

    // Service to set max speed
    this.node.createService(
      'example_interfaces/srv/SetBool',
      'set_safety_mode',
      (request, response) => {
        this.robotConfig.safetyEnabled = request.data;

        const result = response.template;
        result.success = true;
        result.message = `Safety mode set to: ${request.data}`;
        response.send(result);
      }
    );

    console.log('Robot configuration services ready');
    rclnodejs.spin(this.node);
  }
}

const configService = new RobotConfigurationService();
configService.start().catch(console.error);
```

## When to Use Topics vs Services

### Use Topics When:

- ✅ **Continuous data** - Sensor readings, status updates
- ✅ **Multiple consumers** - Many nodes need the same data
- ✅ **Asynchronous** - Publisher doesn't need immediate response
- ✅ **High frequency** - Data published regularly (> 1 Hz)
- ✅ **Fire-and-forget** - Don't care if anyone receives the data

**Examples**: Camera images, laser scans, robot odometry, joint states

### Use Services When:

- ✅ **Request/response** - Need a specific computation or action
- ✅ **Occasional use** - Triggered by events, not continuous
- ✅ **Synchronous** - Need to wait for result before continuing
- ✅ **Configuration** - Setting parameters or modes
- ✅ **Validation** - Need confirmation that action succeeded

**Examples**: Calculating path, setting robot mode, triggering calibration, querying status

### Quick Comparison

| Aspect                 | Topics            | Services                 |
| ---------------------- | ----------------- | ------------------------ |
| **Pattern**            | Publish/Subscribe | Request/Response         |
| **Communication**      | Asynchronous      | Synchronous              |
| **Frequency**          | Continuous/High   | Occasional/On-demand     |
| **Response**           | No response       | Always responds          |
| **Multiple consumers** | Yes               | One server, many clients |
| **Use case**           | Data streams      | Remote procedure calls   |

## Running the Examples

### Topic Examples

Run the publisher and subscriber in separate terminals:

```bash
# Terminal 1 - Start subscriber
cd /path/to/rclnodejs
node example/topics/subscriber/subscription-example.js

# Terminal 2 - Start publisher
cd /path/to/rclnodejs
node example/topics/publisher/publisher-example.js
```

**Expected Output:**

**Subscriber terminal:**

```
Received message: string Hello ROS 0
Received message: string Hello ROS 1
Received message: string Hello ROS 2
```

**Publisher terminal:**

```
Publishing message: Hello ROS 0
Publishing message: Hello ROS 1
Publishing message: Hello ROS 2
```

### Service Examples

Run the service server and client in separate terminals:

```bash
# Terminal 1 - Start service server
cd /path/to/rclnodejs
node example/services/service/service-example.js

# Terminal 2 - Start service client
cd /path/to/rclnodejs
node example/services/client/client-example.js
```

**Expected Output:**

**Service server terminal:**

```
Incoming request: object { a: 45n, b: 67n }
Sending response: object { sum: 112n }
```

**Service client terminal:**

```
Sending: object { a: 45n, b: 67n }
Result: object { sum: 112n }
```

### Using ROS 2 CLI Tools

Monitor your topics and services:

```bash
# List all active topics
ros2 topic list

# Listen to topic messages
ros2 topic echo /topic

# Show topic information
ros2 topic info /topic

# List all services
ros2 service list

# Call a service manually
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 10}"

# Show service type
ros2 service type /add_two_ints
```

### Additional Examples

Explore more patterns in the examples directory:

- **`example/topics/publisher/`** - Various publisher patterns
- **`example/topics/subscriber/`** - Different subscription approaches
- **`example/services/`** - Service implementation examples
- **QoS Examples** - Quality of Service configuration
- **Message Examples** - Working with complex message types

This tutorial covers the fundamental communication patterns that form the backbone of any ROS 2 application. Topics and services provide the foundation for building distributed, modular robotics systems with rclnodejs.
