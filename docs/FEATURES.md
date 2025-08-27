# rclnodejs Features

This document provides an overview of the key features and capabilities available in rclnodejs, the Node.js client library for ROS 2.

## Core ROS 2 Communication

### Publishers and Subscribers

- **Topic Publishing**: Publish messages to ROS 2 topics with full type safety
- **Topic Subscription**: Subscribe to ROS 2 topics with callback-based message handling
- **Raw Message Support**: Access to raw serialized message data for advanced use cases
- **Event Callbacks**: Support for subscription and publisher event handling (incompatible QoS, liveliness lost, etc.)

### Services

- **Service Clients**: Call ROS 2 services with request/response patterns
- **Service Servers**: Create service endpoints to handle incoming requests
- **Service Introspection**: Built-in service call monitoring and debugging capabilities
- **Asynchronous Operations**: Promise-based and callback-based service interactions
- **Type Description Service**: Support for `type_description_interfaces/srv/GetTypeDescription` to retrieve detailed type information for ROS 2 messages, services, and actions

### Actions

- **Action Clients**: Send goals to action servers with feedback and result handling
- **Action Servers**: Implement action endpoints with goal handling, feedback publishing, and result reporting
- **Goal Management**: Full lifecycle management including goal cancellation and preemption
- **Multiple Goal Support**: Handle multiple concurrent goals with individual goal handles

## Node Management

### Node Lifecycle

- **Standard Nodes**: Create and manage regular ROS 2 nodes
- **Lifecycle Nodes**: Support for managed node lifecycle (configure, activate, deactivate, shutdown)
- **Node Options**: Comprehensive node configuration including domain ID, namespace, and parameter overrides
- **ROS Arguments**: Create nodes with ROS command-line arguments for remapping and configuration
- **Global Arguments**: Support for `useGlobalArguments` flag to use process-wide ROS arguments
- **Node Destruction**: Proper cleanup and resource management

### Context Management

- **ROS 2 Context**: Initialize and manage ROS 2 execution context
- **Multi-Context Support**: Support for multiple ROS 2 contexts in the same process
- **Context Configuration**: Custom context initialization with domain ID and security settings

## Quality of Service (QoS)

### QoS Profiles

- **Built-in Profiles**: Access to standard ROS 2 QoS profiles (default, sensor data, services, parameters, etc.)
- **Custom QoS**: Create custom QoS policies for specific requirements
- **QoS Configuration**: Set reliability, durability, history, and depth policies
- **QoS Compatibility**: Automatic QoS negotiation and compatibility checking

### QoS Policies

- **Reliability**: Best effort or reliable message delivery
- **Durability**: Transient local or volatile message persistence
- **History**: Keep last N messages or keep all messages
- **Deadline and Lifespan**: Advanced timing constraints for real-time applications

## Time and Clock Management

### Clock Types

- **System Clock**: Standard system time
- **ROS Time**: ROS simulation time support
- **Steady Clock**: Monotonic clock for timing measurements
- **Time Sources**: Automatic time source management and synchronization

### Timing Features

- **Timer Creation**: High-precision periodic timers with nanosecond resolution
- **Rate Control**: Frequency-based rate limiting for periodic operations
- **Time Conversion**: Utilities for time format conversion and manipulation
- **Simulation Time**: Full support for `/clock` topic and `use_sim_time` parameter

## Parameter System

### Parameter Management

- **Parameter Declaration**: Declare node parameters with type checking and validation
- **Parameter Access**: Get and set parameter values at runtime
- **Parameter Events**: Subscribe to parameter change notifications
- **Parameter Services**: Built-in parameter services for external parameter management

### Parameter Types

- **Primitive Types**: Support for bool, int64, float64, string parameters
- **Array Types**: Support for arrays of primitive types
- **Parameter Descriptors**: Rich parameter metadata including ranges, descriptions, and constraints

## Advanced Features

### Lifecycle Publisher

- **Managed Publishing**: Publishers that only send messages when the lifecycle node is active
- **State Synchronization**: Automatic activation/deactivation with lifecycle node state changes
- **Resource Management**: Efficient resource usage by disabling publishing when inactive

### Guard Conditions

- **Custom Triggers**: Create custom triggerable conditions for the ROS 2 executor
- **Event Synchronization**: Synchronize external events with ROS 2 execution
- **Wake-up Mechanisms**: Programmatically wake up the executor for immediate processing

### Message Generation

- **Dynamic Message Types**: Generate JavaScript message classes from ROS 2 interface definitions
- **IDL Support**: Support for Interface Definition Language (IDL) files
- **TypeScript Declarations**: Automatic generation of TypeScript type definitions
- **Custom Messages**: Support for custom message, service, and action definitions

### ROS 2 Process Management

- **Package Execution**: Programmatically run ROS 2 package executables using `ros2Run()` function
- **Launch File Execution**: Execute ROS 2 launch files programmatically using `ros2Launch()` function
- **Argument Support**: Pass command-line arguments to executables and launch files
- **Input Validation**: Comprehensive validation for package names, executables, and launch files

## Development Tools

### TypeScript Support

- **Full Type Safety**: Complete TypeScript definitions for all APIs
- **Auto-completion**: Rich IDE support with intelligent code completion
- **Type Checking**: Compile-time type checking for ROS 2 message structures
- **Modern JavaScript**: Support for ES modules and modern JavaScript features

### Logging

- **ROS 2 Logging**: Integration with ROS 2's logging infrastructure
- **Log Levels**: Support for DEBUG, INFO, WARN, ERROR, and FATAL log levels
- **Logger Configuration**: Per-node logger configuration and management
- **Console Output**: Formatted console output with timestamps and severity levels

### Introspection and Debugging

- **Service Introspection**: Monitor service calls with detailed request/response logging
- **Graph Introspection**: Access to ROS 2 computation graph information
- **Event Monitoring**: Track publisher/subscriber events and QoS incompatibilities
- **Performance Metrics**: Built-in performance monitoring and profiling capabilities

## Integration Features

### Cross-Platform Support

- **Windows**: Full support for Windows development environments
- **Linux**: Native Linux support with system package integration
- **macOS**: Complete macOS compatibility
- **Docker**: Container-friendly deployment options

### Electron Integration

- **Desktop Applications**: Build cross-platform desktop applications using Electron
- **Web Technologies**: Leverage HTML, CSS, and modern web frameworks
- **3D Visualization**: Integration with WebGL and Three.js for 3D robotics visualization
- **Real-time UIs**: Create responsive user interfaces for robot control and monitoring

### Node.js Ecosystem

- **NPM Integration**: Standard npm package management and dependency resolution
- **Module System**: Support for CommonJS and ES modules
- **Async/Await**: Modern asynchronous programming patterns
- **Event-Driven**: Built on Node.js event-driven architecture for high performance

## Security and Reliability

### Error Handling

- **Exception Safety**: Robust error handling with proper exception propagation
- **Resource Cleanup**: Automatic resource cleanup on node destruction
- **Memory Management**: Efficient memory management with garbage collection integration
- **Graceful Shutdown**: Clean shutdown procedures with proper resource deallocation

### Security Features

- **ROS 2 Security**: Integration with ROS 2 security framework (SROS2)
- **Certificate Management**: Support for X.509 certificates and key management
- **Access Control**: Fine-grained access control for topics, services, and actions
- **Encryption**: Support for DDS security features including encryption and authentication

## Performance Optimization

### Efficient Execution

- **Single-Threaded**: Efficient single-threaded execution model optimized for Node.js
- **Event Loop Integration**: Seamless integration with Node.js event loop
- **Memory Efficiency**: Optimized memory usage with minimal overhead
- **Zero-Copy Operations**: Support for zero-copy message passing where possible

### Scalability

- **Multiple Nodes**: Support for multiple nodes in a single process
- **High-Frequency Operations**: Optimized for high-frequency publishing and subscription
- **Large Message Support**: Efficient handling of large message payloads
- **Connection Management**: Automatic connection management and optimization

---

For detailed API documentation and usage examples, see the [API Documentation](https://robotwebtools.github.io/rclnodejs/docs/index.html) and [Examples](https://github.com/RobotWebTools/rclnodejs/tree/develop/example).
