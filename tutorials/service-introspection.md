# Service Introspection in rclnodejs

Service Introspection is a powerful debugging and monitoring feature in ROS 2 that allows you to observe and analyze service interactions in real-time. This tutorial will guide you through understanding and using service introspection with rclnodejs.

## Table of Contents

- [What is Service Introspection?](#what-is-service-introspection)
- [Requirements](#requirements)
- [How Service Introspection Works](#how-service-introspection-works)
- [Introspection States](#introspection-states)
- [Basic Usage](#basic-usage)
- [Complete Examples](#complete-examples)
- [Monitoring Service Events](#monitoring-service-events)
- [Event Types and Structure](#event-types-and-structure)
- [Advanced Usage](#advanced-usage)
- [Best Practices](#best-practices)
- [Troubleshooting](#troubleshooting)

## What is Service Introspection?

Service Introspection automatically publishes detailed information about service calls (requests and responses) to special topics, enabling you to:

- **Monitor service activity** without modifying client code
- **Debug service interactions** by seeing actual request/response data
- **Analyze system behavior** and performance
- **Log service events** for later analysis
- **Understand service call patterns** in complex systems

When introspection is enabled, ROS 2 automatically creates a special event topic:

```
/your_service_name/_service_event
```

## Requirements

Service Introspection is available in:

- **ROS 2 Iron** and newer distributions
- **rclnodejs** with compatible ROS 2 installation

**Note**: Service introspection is **not available** in ROS 2 Humble and earlier versions.

```javascript
const rclnodejs = require('rclnodejs');

// Check if introspection is supported
if (
  rclnodejs.DistroUtils.getDistroId() <=
  rclnodejs.DistroUtils.getDistroId('humble')
) {
  console.warn(
    'Service introspection is not supported by this version of ROS 2'
  );
}
```

## How Service Introspection Works

When introspection is configured on a service or client, it publishes `service_msgs/msg/ServiceEventInfo` messages containing:

- **Request data** (what was sent to the service)
- **Response data** (what the service returned)
- **Timestamps** (when the interaction occurred)
- **Client information** (who made the request)
- **Event type** (request sent, request received, response sent, response received)

## Introspection States

rclnodejs provides three introspection states:

```javascript
const ServiceIntrospectionStates = rclnodejs.ServiceIntrospectionStates;

console.log(ServiceIntrospectionStates.OFF); // 0 - Disabled
console.log(ServiceIntrospectionStates.METADATA); // 1 - Only metadata (no content)
console.log(ServiceIntrospectionStates.CONTENTS); // 2 - Full request/response data
```

### State Details

| State      | Value | Description                | Use Case                    |
| ---------- | ----- | -------------------------- | --------------------------- |
| `OFF`      | 0     | Introspection disabled     | Production systems, privacy |
| `METADATA` | 1     | Only timing and event info | Performance monitoring      |
| `CONTENTS` | 2     | Full request/response data | Debugging, development      |

## Basic Usage

### Service-Side Introspection

```javascript
const rclnodejs = require('rclnodejs');

rclnodejs.init().then(() => {
  const node = new rclnodejs.Node('service_introspection_example');

  // Create a service
  const service = node.createService(
    'example_interfaces/srv/AddTwoInts',
    'add_two_ints',
    (request, response) => {
      console.log(`Processing: ${request.a} + ${request.b}`);
      const result = response.template;
      result.sum = request.a + request.b;
      response.send(result);
    }
  );

  // Configure introspection (ROS 2 Iron+ only)
  if (
    rclnodejs.DistroUtils.getDistroId() >
    rclnodejs.DistroUtils.getDistroId('humble')
  ) {
    service.configureIntrospection(
      node.getClock(), // Clock for timestamps
      rclnodejs.QoS.profileSystemDefault, // QoS profile
      rclnodejs.ServiceIntrospectionStates.CONTENTS // Include full data
    );
    console.log('Service introspection configured');
  }

  node.spin();
});
```

### Client-Side Introspection

```javascript
const rclnodejs = require('rclnodejs');

rclnodejs.init().then(async () => {
  const node = new rclnodejs.Node('client_introspection_example');

  // Create a client
  const client = node.createClient(
    'example_interfaces/srv/AddTwoInts',
    'add_two_ints'
  );

  // Wait for service to be available
  if (!(await client.waitForService(5000))) {
    console.error('Service not available');
    return;
  }

  // Configure introspection (ROS 2 Iron+ only)
  if (
    rclnodejs.DistroUtils.getDistroId() >
    rclnodejs.DistroUtils.getDistroId('humble')
  ) {
    client.configureIntrospection(
      node.getClock(), // Clock for timestamps
      rclnodejs.QoS.profileSystemDefault, // QoS profile
      rclnodejs.ServiceIntrospectionStates.CONTENTS // Include full data
    );
    console.log('Client introspection configured');
  }

  // Start spinning
  node.spin();

  // Make a service call (note: using BigInt for integer values)
  const request = { a: BigInt(5), b: BigInt(3) };
  client.sendRequest(request, (response) => {
    console.log(`Result: ${request.a} + ${request.b} = ${response.sum}`);
  });
});
```

### Complete Test-Based Example

This example follows the exact patterns used in the official test suite:

```javascript
const rclnodejs = require('rclnodejs');

const DELAY = 1000; // ms

function isServiceIntrospectionSupported() {
  return (
    rclnodejs.DistroUtils.getDistroId() >
    rclnodejs.DistroUtils.getDistroId('humble')
  );
}

function runClient(client, request, delay = DELAY) {
  client.sendRequest(request, (response) => {
    // do nothing - just like in the test
  });

  return new Promise((resolve) => {
    setTimeout(resolve, delay);
  });
}

async function testServiceIntrospection() {
  if (!isServiceIntrospectionSupported()) {
    console.log('Service introspection is not supported in this ROS 2 version');
    return;
  }

  await rclnodejs.init();

  // Create node using the constructor (as in tests)
  const node = new rclnodejs.Node('service_example_node');

  // Create service
  const service = node.createService(
    'example_interfaces/srv/AddTwoInts',
    'add_two_ints',
    (request, response) => {
      let result = response.template;
      result.sum = request.a + request.b;
      response.send(result);
    }
  );

  // Create client
  const client = node.createClient(
    'example_interfaces/srv/AddTwoInts',
    'add_two_ints'
  );

  // Wait for service to be available
  if (!(await client.waitForService(1000))) {
    rclnodejs.shutdown();
    throw new Error('client unable to access service');
  }

  // Create request with BigInt values (as required by interface)
  const request = {
    a: BigInt(Math.floor(Math.random() * 100)),
    b: BigInt(Math.floor(Math.random() * 100)),
  };

  // Set up event monitoring
  const eventQueue = [];
  const serviceEventSubscriber = node.createSubscription(
    'example_interfaces/srv/AddTwoInts_Event',
    '/add_two_ints/_service_event',
    function (event) {
      eventQueue.push(event);
    }
  );

  // Configure introspection for both service and client
  const QOS = rclnodejs.QoS.profileSystemDefault;

  service.configureIntrospection(
    node.getClock(),
    QOS,
    rclnodejs.ServiceIntrospectionStates.CONTENTS
  );

  client.configureIntrospection(
    node.getClock(),
    QOS,
    rclnodejs.ServiceIntrospectionStates.CONTENTS
  );

  // Start spinning
  node.spin();

  // Make service call using the test pattern
  console.log(`Sending request: ${request.a} + ${request.b}`);
  await runClient(client, request);

  // Verify results (like in the test)
  console.log(`Total events received: ${eventQueue.length}`);

  // With both client and service introspection enabled, expect 4 events:
  // 0: REQUEST_SENT (client) - event_type: 0
  // 1: REQUEST_RECEIVED (service) - event_type: 1
  // 2: RESPONSE_SENT (service) - event_type: 2
  // 3: RESPONSE_RECEIVED (client) - event_type: 3

  if (eventQueue.length === 4) {
    console.log('✓ Received expected 4 events');

    for (let i = 0; i < 4; i++) {
      console.log(
        `Event ${i}: Type ${eventQueue[i].info.event_type} (expected: ${i})`
      );

      if (i < 2) {
        // Request events (0, 1) should have request data, no response data
        console.log(
          `  - Request data: ${eventQueue[i].request.length > 0 ? 'Present' : 'Missing'}`
        );
        console.log(
          `  - Response data: ${eventQueue[i].response.length === 0 ? 'Absent (correct)' : 'Present (unexpected)'}`
        );
      } else {
        // Response events (2, 3) should have response data, no request data
        console.log(
          `  - Request data: ${eventQueue[i].request.length === 0 ? 'Absent (correct)' : 'Present (unexpected)'}`
        );
        console.log(
          `  - Response data: ${eventQueue[i].response.length > 0 ? 'Present' : 'Missing'}`
        );
      }
    }
  } else {
    console.log(`✗ Expected 4 events, got ${eventQueue.length}`);
  }

  rclnodejs.shutdown();
}

// Run the test
testServiceIntrospection().catch(console.error);
```

### Different Introspection Configurations

Based on the test patterns, here are the different configuration scenarios with expected results:

```javascript
const rclnodejs = require('rclnodejs');
const QOS = rclnodejs.QoS.profileSystemDefault;

// Test 1: Both client and service with CONTENTS
// Results in 4 events (all request/response data included)
service.configureIntrospection(
  clock,
  QOS,
  rclnodejs.ServiceIntrospectionStates.CONTENTS
);
client.configureIntrospection(
  clock,
  QOS,
  rclnodejs.ServiceIntrospectionStates.CONTENTS
);
// Expected: 4 events with event_types [0, 1, 2, 3]
// Events 0,1 have request data; Events 2,3 have response data

// Test 2: Service-only with METADATA
// Results in 2 events (REQUEST_RECEIVED=1, RESPONSE_SENT=2, no data)
service.configureIntrospection(
  clock,
  QOS,
  rclnodejs.ServiceIntrospectionStates.METADATA
);
// client introspection not configured
// Expected: 2 events with event_types [1, 2]
// Both events have no request or response data (METADATA only)

// Test 3: Client-only with METADATA
// Results in 2 events (REQUEST_SENT=0, RESPONSE_RECEIVED=3, no data)
client.configureIntrospection(
  clock,
  QOS,
  rclnodejs.ServiceIntrospectionStates.METADATA
);
// service introspection not configured
// Expected: 2 events with event_types [0, 3]
// Both events have no request or response data (METADATA only)

// Test 4: Mixed configurations
// Service CONTENTS + Client OFF = 2 events with service-side data
service.configureIntrospection(
  clock,
  QOS,
  rclnodejs.ServiceIntrospectionStates.CONTENTS
);
client.configureIntrospection(
  clock,
  QOS,
  rclnodejs.ServiceIntrospectionStates.OFF
);
// Expected: 2 events with event_types [1, 2]
// Event 1 has request data, Event 2 has response data

// Service OFF + Client CONTENTS = 2 events with client-side data
service.configureIntrospection(
  clock,
  QOS,
  rclnodejs.ServiceIntrospectionStates.OFF
);
client.configureIntrospection(
  clock,
  QOS,
  rclnodejs.ServiceIntrospectionStates.CONTENTS
);
// Expected: 2 events with event_types [0, 3]
// Event 0 has request data, Event 3 has response data

// Test 5: Both OFF = No events
service.configureIntrospection(
  clock,
  QOS,
  rclnodejs.ServiceIntrospectionStates.OFF
);
client.configureIntrospection(
  clock,
  QOS,
  rclnodejs.ServiceIntrospectionStates.OFF
);
// Expected: 0 events

// Test 6: No configuration = No events
// If neither service nor client calls configureIntrospection()
// Expected: 0 events
```

## Complete Examples

### Full Service and Client with Introspection

```javascript
const rclnodejs = require('rclnodejs');

async function runIntrospectionExample() {
  await rclnodejs.init();

  // Create nodes
  const serviceNode = new rclnodejs.Node('introspection_service_node');
  const clientNode = new rclnodejs.Node('introspection_client_node');
  const monitorNode = new rclnodejs.Node('introspection_monitor_node');

  // Create service
  const service = serviceNode.createService(
    'example_interfaces/srv/AddTwoInts',
    'add_two_ints_introspection',
    (request, response) => {
      console.log(`Service processing: ${request.a} + ${request.b}`);
      const result = response.template;
      result.sum = request.a + request.b;
      response.send(result);
    }
  );

  // Create client
  const client = clientNode.createClient(
    'example_interfaces/srv/AddTwoInts',
    'add_two_ints_introspection'
  );

  // Create event monitor subscription
  const eventSubscription = monitorNode.createSubscription(
    'example_interfaces/srv/AddTwoInts_Event',
    '/add_two_ints_introspection/_service_event',
    (eventMsg) => {
      console.log('=== Service Event ===');
      console.log(`Event Type: ${getEventTypeName(eventMsg.info.event_type)}`);
      console.log(
        `Timestamp: ${eventMsg.info.stamp.sec}.${eventMsg.info.stamp.nanosec}`
      );
      console.log(`Sequence: ${eventMsg.info.sequence_number}`);

      if (eventMsg.request && eventMsg.request.length > 0) {
        console.log(
          `Request: a=${eventMsg.request[0].a}, b=${eventMsg.request[0].b}`
        );
      }

      if (eventMsg.response && eventMsg.response.length > 0) {
        console.log(`Response: sum=${eventMsg.response[0].sum}`);
      }

      console.log('====================\n');
    }
  );

  // Configure introspection if supported
  if (
    rclnodejs.DistroUtils.getDistroId() >
    rclnodejs.DistroUtils.getDistroId('humble')
  ) {
    const clock = serviceNode.getClock();
    const qos = rclnodejs.QoS.profileSystemDefault;

    // Configure both service and client introspection
    service.configureIntrospection(
      clock,
      qos,
      rclnodejs.ServiceIntrospectionStates.CONTENTS
    );
    client.configureIntrospection(
      clock,
      qos,
      rclnodejs.ServiceIntrospectionStates.CONTENTS
    );

    console.log('Introspection configured for both service and client');
  } else {
    console.log('Introspection not supported in this ROS 2 version');
  }

  // Wait for service
  if (!(await client.waitForService(5000))) {
    console.error('Service not available');
    return;
  }

  // Start spinning all nodes
  serviceNode.spin();
  clientNode.spin();
  monitorNode.spin();

  // Make multiple service calls
  for (let i = 0; i < 3; i++) {
    const request = {
      a: BigInt(Math.floor(Math.random() * 100)),
      b: BigInt(Math.floor(Math.random() * 100)),
    };

    console.log(`\nSending request ${i + 1}: ${request.a} + ${request.b}`);

    client.sendRequest(request, (response) => {
      console.log(`Received response ${i + 1}: ${response.sum}`);
    });

    // Wait between calls
    await new Promise((resolve) => setTimeout(resolve, 2000));
  }
}

function getEventTypeName(eventType) {
  const eventTypes = {
    0: 'REQUEST_SENT',
    1: 'REQUEST_RECEIVED',
    2: 'RESPONSE_SENT',
    3: 'RESPONSE_RECEIVED',
  };
  return eventTypes[eventType] || `UNKNOWN(${eventType})`;
}

// Run the example
runIntrospectionExample().catch(console.error);
```

## Monitoring Service Events

### Using Command Line Tools

Monitor service events from the command line:

```bash
# List all topics to find service event topics
ros2 topic list | grep _service_event

# Monitor events for a specific service
ros2 topic echo "/add_two_ints/_service_event"

# Monitor with message info
ros2 topic echo --include-types "/add_two_ints/_service_event"

# Check event publishing frequency
ros2 topic hz "/add_two_ints/_service_event"
```

### Programmatic Monitoring

```javascript
const rclnodejs = require('rclnodejs');

async function createServiceEventMonitor(serviceName, serviceType) {
  await rclnodejs.init();

  const node = new rclnodejs.Node('service_event_monitor');

  // Subscribe to service events
  const eventSubscription = node.createSubscription(
    `${serviceType}_Event`,
    `/${serviceName}/_service_event`,
    (eventMsg) => {
      const info = eventMsg.info;
      const timestamp = `${info.stamp.sec}.${String(info.stamp.nanosec).padStart(9, '0')}`;

      console.log(
        `[${timestamp}] ${getEventTypeName(info.event_type)} (seq: ${info.sequence_number})`
      );

      // Log request data if available
      if (eventMsg.request && eventMsg.request.length > 0) {
        console.log(`  Request: ${JSON.stringify(eventMsg.request[0])}`);
      }

      // Log response data if available
      if (eventMsg.response && eventMsg.response.length > 0) {
        console.log(`  Response: ${JSON.stringify(eventMsg.response[0])}`);
      }
    }
  );

  node.spin();
  console.log(`Monitoring service events for: ${serviceName}`);
}

// Monitor the add_two_ints service
createServiceEventMonitor('add_two_ints', 'example_interfaces/srv/AddTwoInts');
```

## Event Types and Structure

### Event Types

Service introspection publishes four types of events:

| Event Type          | Value | Description              | Published By |
| ------------------- | ----- | ------------------------ | ------------ |
| `REQUEST_SENT`      | 0     | Client sent request      | Client       |
| `REQUEST_RECEIVED`  | 1     | Service received request | Service      |
| `RESPONSE_SENT`     | 2     | Service sent response    | Service      |
| `RESPONSE_RECEIVED` | 3     | Client received response | Client       |

### Event Message Structure

```javascript
// Example service event message structure
const serviceEvent = {
  info: {
    event_type: 1, // REQUEST_RECEIVED
    stamp: {
      // Timestamp
      sec: 1234567890,
      nanosec: 123456789,
    },
    client_gid: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16],
    sequence_number: BigInt(42), // Unique sequence number
  },
  request: [
    // Request data (if CONTENTS state)
    {
      a: BigInt(5),
      b: BigInt(3),
    },
  ],
  response: [], // Response data (empty for request events)
};
```

## Advanced Usage

### Different Introspection States

```javascript
async function demonstrateIntrospectionStates() {
  await rclnodejs.init();

  const node = new rclnodejs.Node('introspection_states_demo');

  const service = node.createService(
    'example_interfaces/srv/AddTwoInts',
    'introspection_demo',
    (request, response) => {
      const result = response.template;
      result.sum = request.a + request.b;
      response.send(result);
    }
  );

  const clock = node.getClock();
  const qos = rclnodejs.QoS.profileSystemDefault;

  // Demonstrate different states
  console.log('=== Testing METADATA state ===');
  service.configureIntrospection(
    clock,
    qos,
    rclnodejs.ServiceIntrospectionStates.METADATA
  );
  // Events will contain timing info but no request/response data

  // Wait, then switch to CONTENTS
  setTimeout(() => {
    console.log('=== Switching to CONTENTS state ===');
    service.configureIntrospection(
      clock,
      qos,
      rclnodejs.ServiceIntrospectionStates.CONTENTS
    );
    // Events will now contain full request/response data
  }, 10000);

  // Later, turn off introspection
  setTimeout(() => {
    console.log('=== Turning OFF introspection ===');
    service.configureIntrospection(
      clock,
      qos,
      rclnodejs.ServiceIntrospectionStates.OFF
    );
    // No more events will be published
  }, 20000);

  node.spin();
}
```

### Custom QoS for Introspection

```javascript
// Configure introspection with custom QoS
const customQoS = {
  reliability: rclnodejs.QoS.ReliabilityPolicy.RELIABLE,
  durability: rclnodejs.QoS.DurabilityPolicy.TRANSIENT_LOCAL,
  depth: 100, // Keep last 100 messages
};

service.configureIntrospection(
  node.getClock(),
  customQoS,
  rclnodejs.ServiceIntrospectionStates.CONTENTS
);
```

### Performance Analysis with Introspection

```javascript
class ServicePerformanceAnalyzer {
  constructor(serviceName, serviceType) {
    this.serviceName = serviceName;
    this.serviceType = serviceType;
    this.requestTimes = new Map();
    this.statistics = {
      totalCalls: 0,
      totalResponseTime: 0,
      minResponseTime: Infinity,
      maxResponseTime: 0,
    };
  }

  async start() {
    await rclnodejs.init();

    const node = new rclnodejs.Node('performance_analyzer');

    node.createSubscription(
      `${this.serviceType}_Event`,
      `/${this.serviceName}/_service_event`,
      (eventMsg) => this.processEvent(eventMsg)
    );

    rclnodejs.spin(node);

    // Print statistics every 10 seconds
    setInterval(() => this.printStatistics(), 10000);
  }

  processEvent(eventMsg) {
    const info = eventMsg.info;
    const timestamp = info.stamp.sec * 1000 + info.stamp.nanosec / 1000000; // Convert to ms
    const seqNum = info.sequence_number.toString();

    switch (info.event_type) {
      case 1: // REQUEST_RECEIVED
        this.requestTimes.set(seqNum, timestamp);
        break;

      case 2: // RESPONSE_SENT
        if (this.requestTimes.has(seqNum)) {
          const requestTime = this.requestTimes.get(seqNum);
          const responseTime = timestamp - requestTime;

          this.updateStatistics(responseTime);
          this.requestTimes.delete(seqNum);
        }
        break;
    }
  }

  updateStatistics(responseTime) {
    this.statistics.totalCalls++;
    this.statistics.totalResponseTime += responseTime;
    this.statistics.minResponseTime = Math.min(
      this.statistics.minResponseTime,
      responseTime
    );
    this.statistics.maxResponseTime = Math.max(
      this.statistics.maxResponseTime,
      responseTime
    );
  }

  printStatistics() {
    const stats = this.statistics;
    if (stats.totalCalls === 0) {
      console.log('No service calls recorded yet');
      return;
    }

    const avgResponseTime = stats.totalResponseTime / stats.totalCalls;

    console.log(`\n=== Performance Statistics for ${this.serviceName} ===`);
    console.log(`Total calls: ${stats.totalCalls}`);
    console.log(`Average response time: ${avgResponseTime.toFixed(2)} ms`);
    console.log(`Min response time: ${stats.minResponseTime.toFixed(2)} ms`);
    console.log(`Max response time: ${stats.maxResponseTime.toFixed(2)} ms`);
    console.log('=============================================\n');
  }
}

// Usage
const analyzer = new ServicePerformanceAnalyzer(
  'add_two_ints',
  'example_interfaces/srv/AddTwoInts'
);
analyzer.start();
```

## Best Practices

### 1. Use Appropriate Introspection States

```javascript
// For production monitoring (minimal overhead)
service.configureIntrospection(clock, qos, ServiceIntrospectionStates.METADATA);

// For development and debugging (includes full data)
service.configureIntrospection(clock, qos, ServiceIntrospectionStates.CONTENTS);

// For sensitive data or high-performance requirements
service.configureIntrospection(clock, qos, ServiceIntrospectionStates.OFF);
```

### 2. Configure QoS Appropriately

```javascript
// For debugging (reliable delivery)
const debugQoS = {
  reliability: rclnodejs.QoS.ReliabilityPolicy.RELIABLE,
  durability: rclnodejs.QoS.DurabilityPolicy.VOLATILE,
  depth: 50,
};

// For performance monitoring (best effort, larger buffer)
const performanceQoS = {
  reliability: rclnodejs.QoS.ReliabilityPolicy.BEST_EFFORT,
  durability: rclnodejs.QoS.DurabilityPolicy.VOLATILE,
  depth: 1000,
};
```

### 3. Handle Version Compatibility

```javascript
function configureIntrospectionSafely(service, clock, qos, state) {
  if (
    rclnodejs.DistroUtils.getDistroId() >
    rclnodejs.DistroUtils.getDistroId('humble')
  ) {
    try {
      service.configureIntrospection(clock, qos, state);
      console.log('Introspection configured successfully');
      return true;
    } catch (error) {
      console.warn('Failed to configure introspection:', error.message);
      return false;
    }
  } else {
    console.log('Introspection not supported in this ROS 2 version');
    return false;
  }
}
```

### 4. Clean Up Resources

```javascript
function createManagedIntrospectionService() {
  let service;
  let node;

  async function start() {
    await rclnodejs.init();
    node = new rclnodejs.Node('managed_service');

    service = node.createService(
      'example_interfaces/srv/AddTwoInts',
      'managed_service',
      handleRequest
    );

    configureIntrospectionSafely(
      service,
      node.getClock(),
      rclnodejs.QoS.profileSystemDefault,
      rclnodejs.ServiceIntrospectionStates.CONTENTS
    );

    rclnodejs.spin(node);
  }

  function stop() {
    if (service) {
      // Disable introspection before cleanup
      try {
        service.configureIntrospection(
          node.getClock(),
          rclnodejs.QoS.profileSystemDefault,
          rclnodejs.ServiceIntrospectionStates.OFF
        );
      } catch (error) {
        console.warn('Failed to disable introspection:', error.message);
      }
    }

    rclnodejs.shutdown();
  }

  function handleRequest(request, response) {
    const result = response.template;
    result.sum = request.a + request.b;
    response.send(result);
  }

  // Handle cleanup on exit
  process.on('SIGINT', stop);
  process.on('SIGTERM', stop);

  return { start, stop };
}
```

## Code Pattern Summary

Based on the official test implementation, use these patterns for reliable Service Introspection usage:

### Required Imports and Setup

```javascript
const rclnodejs = require('rclnodejs');

// Access constants directly from rclnodejs
const ServiceIntrospectionStates = rclnodejs.ServiceIntrospectionStates;
const QOS = rclnodejs.QoS.profileSystemDefault;

// Check compatibility
function isServiceIntrospectionSupported() {
  return (
    rclnodejs.DistroUtils.getDistroId() >
    rclnodejs.DistroUtils.getDistroId('humble')
  );
}
```

### Node Creation and Service Setup

```javascript
// Use the constructor pattern (as in tests)
const node = new rclnodejs.Node('node_name');

// Create service and client
const service = node.createService('srv_type', 'service_name', callback);
const client = node.createClient('srv_type', 'service_name');

// Always wait for service availability
if (!(await client.waitForService(1000))) {
  throw new Error('client unable to access service');
}

// Start spinning
node.spin();
```

### Introspection Configuration

```javascript
// Configure introspection (only if supported)
if (isServiceIntrospectionSupported()) {
  service.configureIntrospection(
    node.getClock(),
    QOS,
    ServiceIntrospectionStates.CONTENTS // or METADATA or OFF
  );

  client.configureIntrospection(
    node.getClock(),
    QOS,
    ServiceIntrospectionStates.CONTENTS // or METADATA or OFF
  );
}
```

### Event Monitoring Setup

```javascript
const eventQueue = [];
const serviceEventSubscriber = node.createSubscription(
  'your_service_type_Event', // Note: _Event suffix
  '/service_name/_service_event',
  function (event) {
    eventQueue.push(event);
  }
);
```

### Service Call Pattern (Test Style)

```javascript
function runClient(client, request, delay = 1000) {
  client.sendRequest(request, (response) => {
    // Process response or do nothing
  });

  return new Promise((resolve) => {
    setTimeout(resolve, delay);
  });
}

// Use BigInt for integer fields
const request = {
  a: BigInt(5),
  b: BigInt(3),
};

// Make call and wait
await runClient(client, request);
```

### Event Validation Pattern

```javascript
// Expected event counts by configuration:
// Both CONTENTS: 4 events (types 0,1,2,3)
// Service METADATA only: 2 events (types 1,2)
// Client METADATA only: 2 events (types 0,3)
// Both OFF or unconfigured: 0 events

console.log(`Total events: ${eventQueue.length}`);

eventQueue.forEach((event, index) => {
  console.log(`Event ${index}: type=${event.info.event_type}`);
  console.log(`  Request data: ${event.request.length > 0}`);
  console.log(`  Response data: ${event.response.length > 0}`);
});
```

## Troubleshooting

### Common Issues

#### 1. "Service introspection is not supported"

**Problem**: Warning message about unsupported introspection.

**Solution**:

- Ensure you're using ROS 2 Iron or newer
- Check your ROS 2 installation
- Use the compatibility check function:

```javascript
function isServiceIntrospectionSupported() {
  return (
    rclnodejs.DistroUtils.getDistroId() >
    rclnodejs.DistroUtils.getDistroId('humble')
  );
}
```

```bash
# Check ROS 2 version
ros2 --version

# Verify introspection support
ros2 service list --help | grep -i introspect
```

#### 2. No introspection events published

**Possible causes**:

- Introspection not configured
- Wrong topic name
- QoS mismatch

**Solutions**:

```javascript
// Verify topic exists
// Use: ros2 topic list | grep _service_event

// Check if introspection is actually configured
console.log('Configuring introspection...');
const success = configureIntrospectionSafely(service, clock, qos, state);
if (!success) {
  console.error('Introspection configuration failed');
}

// Verify event topic exists after configuration
setTimeout(() => {
  // Check with: ros2 topic list | grep _service_event
}, 1000);
```

#### 3. Missing request/response data in events

**Problem**: Events show up but request/response arrays are empty.

**Solution**: Ensure you're using `CONTENTS` state, not `METADATA`.

```javascript
// Wrong - only metadata
service.configureIntrospection(clock, qos, ServiceIntrospectionStates.METADATA);

// Correct - includes content
service.configureIntrospection(clock, qos, ServiceIntrospectionStates.CONTENTS);
```

#### 4. Performance impact

**Problem**: Introspection affecting service performance.

**Solutions**:

- Use `METADATA` state instead of `CONTENTS`
- Configure appropriate QoS with `BEST_EFFORT` reliability
- Disable introspection in production

```javascript
// Minimal performance impact
const lightweightQoS = {
  reliability: rclnodejs.QoS.ReliabilityPolicy.BEST_EFFORT,
  durability: rclnodejs.QoS.DurabilityPolicy.VOLATILE,
  depth: 10,
};

service.configureIntrospection(
  clock,
  lightweightQoS,
  ServiceIntrospectionStates.METADATA
);
```

### Debugging Tips

#### 1. Verify introspection is working

```bash
# Terminal 1: Run your service with introspection
node your_service.js

# Terminal 2: Check if event topic exists
ros2 topic list | grep _service_event

# Terminal 3: Monitor events
ros2 topic echo "/your_service/_service_event"

# Terminal 4: Make a service call
ros2 service call /your_service example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"
```

#### 2. Log introspection configuration

```javascript
function logIntrospectionConfig(service, serviceName) {
  console.log(`Introspection configuration for ${serviceName}:`);
  console.log(`- ROS 2 Version: ${rclnodejs.DistroUtils.getDistroId()}`);
  console.log(
    `- Introspection supported: ${rclnodejs.DistroUtils.getDistroId() > rclnodejs.DistroUtils.getDistroId('humble')}`
  );
  console.log(`- Expected event topic: /${serviceName}/_service_event`);
}
```

## Conclusion

Service Introspection in rclnodejs provides powerful capabilities for monitoring, debugging, and analyzing ROS 2 service interactions. By understanding the different introspection states, event types, and best practices, you can effectively use this feature to:

- Debug service communication issues
- Monitor system performance
- Analyze service usage patterns
- Log service interactions for compliance or auditing

Remember to use introspection judiciously in production systems, considering the performance impact and data sensitivity requirements of your application.

For more information, see:

- [ROS 2 Service Introspection Documentation](https://docs.ros.org/en/kilted/Tutorials/Demos/Service-Introspection.html)
- [rclnodejs API Documentation](https://robotwebtools.github.io/rclnodejs/docs/index.html)
- [Example service code](../example/services/service/service-example.js)
- [Introspection test cases](../test/test-service-introspection.js)
