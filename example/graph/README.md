# ROS 2 Graph Examples

This directory contains examples demonstrating ROS 2 graph introspection using rclnodejs. These examples show how to programmatically discover and inspect the ROS 2 computational graph, including nodes, topics, services, and their relationships.

## Overview

The ROS 2 computational graph represents the runtime structure of a ROS 2 system, showing:

- **Nodes**: Computational processes that perform specific tasks
- **Topics**: Named communication channels for publish-subscribe messaging
- **Services**: Request-response communication endpoints
- **Actions**: Long-running task coordination mechanisms
- **Parameters**: Configuration values for nodes

Graph introspection allows you to:

- Discover what nodes are running in the system
- Identify available topics and their message types
- Find services and their interfaces
- Understand the communication relationships between nodes
- Debug and monitor system architecture

## Graph Example

### ROS Graph Discovery (`ros-graph-example.js`)

**Purpose**: Demonstrates comprehensive ROS 2 graph introspection capabilities.

#### Functionality

This example creates a complete ROS 2 system with multiple nodes and then introspects the graph to display:

1. **Node Creation**: Creates several nodes with different communication patterns:
   - `publisher_node` (namespace: `ns1`) - publishes to a topic
   - `subscriber_node` (namespace: `ns1`) - subscribes to a topic
   - `service_node` (namespace: `ns1`) - provides a service
   - `client_node` (namespace: `ns2`) - creates a service client
   - `ros_graph_display_node` (namespace: `ns1`) - performs graph introspection

2. **Graph Introspection**: Uses various discovery methods to analyze the system:
   - Node discovery and namespace listing
   - Topic discovery with message types
   - Service discovery with service types
   - Publisher-subscriber relationships by node
   - Service-client relationships by node

#### Communication Setup

- **Topic Communication**: `publisher_node` and `subscriber_node` communicate via `topic` using `std_msgs/msg/String`
- **Service Communication**: `service_node` provides `add_two_ints` service using `example_interfaces/srv/AddTwoInts`
- **Cross-Namespace**: `client_node` in `ns2` can discover and connect to services in `ns1`

#### Features Demonstrated

- **Multi-namespace architecture**: Nodes in different namespaces (`ns1`, `ns2`)
- **Complete communication patterns**: Publishers, subscribers, services, clients
- **Graph discovery APIs**: All major introspection methods
- **Structured output**: JSON-formatted relationship mapping
- **Real-time introspection**: Live discovery of active system components

#### Run Command

```bash
node ros-graph-example.js
```

## Sample Output

When you run the example, you'll see structured output showing the complete ROS 2 graph:

### Expected Output Structure

```
This example creates the following nodes and outputs the corresponding ROS2 graph:
  publisher_node
  subscriber_node
  service_node
  ros_graph_display_node

NODES
[
  '/ns1/publisher_node',
  '/ns1/subscriber_node',
  '/ns1/service_node',
  '/ns2/client_node',
  '/ns1/ros_graph_display_node'
]

TOPICS & TYPES
[
  { name: '/ns1/topic', types: ['std_msgs/msg/String'] },
  { name: '/parameter_events', types: ['rcl_interfaces/msg/ParameterEvent'] },
  { name: '/rosout', types: ['rcl_interfaces/msg/Log'] }
]

SERVICES & TYPES
[
  { name: '/ns1/add_two_ints', types: ['example_interfaces/srv/AddTwoInts'] },
  { name: '/ns1/publisher_node/describe_parameters', types: ['rcl_interfaces/srv/DescribeParameters'] },
  // ... other built-in node services
]

PUBLISHERS BY NODE
[
  {
    "node": { "name": "publisher_node", "namespace": "/ns1" },
    "info": [
      { "name": "/ns1/topic", "types": ["std_msgs/msg/String"] }
    ]
  },
  // ... other nodes
]

SUBSCRIPTIONS BY NODE
[
  {
    "node": { "name": "subscriber_node", "namespace": "/ns1" },
    "info": [
      { "name": "/ns1/topic", "types": ["std_msgs/msg/String"] }
    ]
  },
  // ... other nodes
]

SERVICES BY NODE
[
  {
    "node": { "name": "service_node", "namespace": "/ns1" },
    "info": [
      { "name": "/ns1/add_two_ints", "types": ["example_interfaces/srv/AddTwoInts"] }
    ]
  },
  // ... other nodes with built-in services
]

CLIENTS BY NODE
[
  { "name": "/ns1/add_two_ints", "types": ["example_interfaces/srv/AddTwoInts"] }
]
```

## Key Graph Introspection APIs

The example demonstrates these important rclnodejs graph discovery methods:

### Node Discovery

- **`getNodeNames()`**: Returns list of all node names in the system
- **`getNodeNamesAndNamespaces()`**: Returns nodes with their namespace information

### Topic Discovery

- **`getTopicNamesAndTypes()`**: Lists all topics with their message types
- **`getPublisherNamesAndTypesByNode(name, namespace)`**: Shows what topics a specific node publishes
- **`getSubscriptionNamesAndTypesByNode(name, namespace)`**: Shows what topics a specific node subscribes to

### Service Discovery

- **`getServiceNamesAndTypes()`**: Lists all services with their interface types
- **`getServiceNamesAndTypesByNode(name, namespace)`**: Shows what services a specific node provides
- **`getClientNamesAndTypesByNode(name, namespace)`**: Shows what services a specific node uses as a client

## Understanding the Output

### Namespace Organization

- **`/ns1`**: Contains publisher, subscriber, service, and display nodes
- **`/ns2`**: Contains the client node
- **Cross-namespace communication**: Client in `ns2` can access services in `ns1`

### Built-in Topics and Services

ROS 2 automatically creates several system topics and services:

- **`/parameter_events`**: Parameter change notifications
- **`/rosout`**: Logging messages
- **Node services**: Each node gets automatic parameter and lifecycle services

### Communication Patterns

- **Publisher-Subscriber**: `publisher_node` → `/ns1/topic` ← `subscriber_node`
- **Service-Client**: `client_node` → `/ns1/add_two_ints` ← `service_node`

## Use Cases for Graph Introspection

### System Monitoring

```javascript
// Check if a specific node is running
const nodes = node.getNodeNames();
const isNodeRunning = nodes.includes('/my_namespace/my_node');

// Monitor topic activity
const topics = node.getTopicNamesAndTypes();
const activeTopics = topics.filter((topic) =>
  topic.name.startsWith('/sensors')
);
```

### Dynamic Service Discovery

```javascript
// Find all available services
const services = node.getServiceNamesAndTypes();
const mathServices = services.filter((service) =>
  service.types.includes('example_interfaces/srv/AddTwoInts')
);
```

### Architecture Validation

```javascript
// Verify expected communication patterns
const nodeInfo = node.getNodeNamesAndNamespaces();
for (const nodeData of nodeInfo) {
  const publishers = node.getPublisherNamesAndTypesByNode(
    nodeData.name,
    nodeData.namespace
  );
  const subscribers = node.getSubscriptionNamesAndTypesByNode(
    nodeData.name,
    nodeData.namespace
  );
  // Validate expected patterns
}
```

## Comparing with ROS 2 CLI Tools

The programmatic introspection shown in this example provides similar information to ROS 2 command-line tools:

| rclnodejs Method                       | Equivalent ROS 2 CLI    |
| -------------------------------------- | ----------------------- |
| `getNodeNames()`                       | `ros2 node list`        |
| `getTopicNamesAndTypes()`              | `ros2 topic list -t`    |
| `getServiceNamesAndTypes()`            | `ros2 service list -t`  |
| `getPublisherNamesAndTypesByNode()`    | `ros2 node info <node>` |
| `getSubscriptionNamesAndTypesByNode()` | `ros2 node info <node>` |

## Advanced Graph Analysis

### Building Communication Maps

```javascript
// Create a communication graph
const communicationMap = new Map();
const nodes = node.getNodeNamesAndNamespaces();

for (const nodeData of nodes) {
  const pubs = node.getPublisherNamesAndTypesByNode(
    nodeData.name,
    nodeData.namespace
  );
  const subs = node.getSubscriptionNamesAndTypesByNode(
    nodeData.name,
    nodeData.namespace
  );

  communicationMap.set(`${nodeData.namespace}/${nodeData.name}`, {
    publishes: pubs,
    subscribes: subs,
  });
}
```

### Topic Connectivity Analysis

```javascript
// Find which nodes communicate via topics
const topics = node.getTopicNamesAndTypes();
const nodes = node.getNodeNamesAndNamespaces();

for (const topic of topics) {
  const publishers = [];
  const subscribers = [];

  for (const nodeData of nodes) {
    const pubs = node.getPublisherNamesAndTypesByNode(
      nodeData.name,
      nodeData.namespace
    );
    const subs = node.getSubscriptionNamesAndTypesByNode(
      nodeData.name,
      nodeData.namespace
    );

    if (pubs.some((p) => p.name === topic.name)) publishers.push(nodeData);
    if (subs.some((s) => s.name === topic.name)) subscribers.push(nodeData);
  }

  console.log(
    `Topic ${topic.name}: ${publishers.length} publishers, ${subscribers.length} subscribers`
  );
}
```

## Notes

- Graph introspection provides a snapshot of the current system state
- Nodes and communication endpoints can appear/disappear dynamically
- The example creates a static graph for demonstration purposes
- In real systems, you might want to poll these APIs periodically to track changes
- Namespace handling is important for multi-robot or complex systems
- Built-in ROS 2 services and topics are automatically included in discovery results
- Graph introspection is useful for debugging communication issues and system architecture validation
