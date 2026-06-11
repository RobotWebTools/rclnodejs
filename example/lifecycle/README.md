# ROS 2 Lifecycle Examples

This directory contains examples demonstrating ROS 2 lifecycle node functionality using rclnodejs. Lifecycle nodes provide a standardized state machine for managing node behavior and resources throughout their operational lifetime.

## Overview

ROS 2 lifecycle nodes implement a well-defined state machine that allows for:

- **Controlled Startup**: Systematic initialization and configuration
- **Resource Management**: Proper allocation and deallocation of resources
- **State Monitoring**: External supervision and control of node states
- **Graceful Shutdown**: Clean termination and cleanup procedures
- **Error Recovery**: Standardized error handling and recovery mechanisms

Lifecycle nodes are particularly useful for:

- System components that require careful initialization
- Nodes that manage hardware resources
- Critical system services that need supervised startup/shutdown
- Applications requiring coordinated state management

## Lifecycle State Machine

The ROS 2 lifecycle state machine includes these primary states:

- **Unconfigured**: Initial state, no resources allocated
- **Inactive**: Configured but not actively processing
- **Active**: Fully operational and processing data
- **Finalized**: Cleaned up and ready for destruction

State transitions are triggered by these transitions:

- **configure**: Unconfigured → Inactive
- **activate**: Inactive → Active
- **deactivate**: Active → Inactive
- **shutdown**: Any state → Finalized
- **cleanup**: Inactive → Unconfigured

## Lifecycle Example

### Lifecycle Node with Countdown (`lifecycle-node-example.mjs`)

**Purpose**: Demonstrates a complete lifecycle node implementation with automated state management.

#### Functionality

This example creates a lifecycle node that:

1. **Countdown Publisher**: Publishes countdown messages from 5 to 0
2. **Self-Monitoring**: Subscribes to its own messages to monitor progress
3. **Automatic Shutdown**: Deactivates and shuts down when countdown reaches 0
4. **State Callbacks**: Implements all major lifecycle transition callbacks
5. **Resource Management**: Properly manages publishers, subscribers, and timers

#### Architecture

- **Node Name**: `test_node`
- **Topic**: `test`
- **Message Type**: `std_msgs/msg/String`
- **Timer Interval**: 1 second
- **Countdown Range**: 5 down to 0

#### State Transition Flow

1. **Initialize**: Create lifecycle node and register callbacks
2. **Configure**: Create publisher and subscriber
3. **Activate**: Activate publisher and start countdown timer
4. **Monitor**: Watch for countdown completion
5. **Deactivate**: Stop timer and deactivate publisher when countdown reaches 0
6. **Shutdown**: Clean up resources and exit

#### Features Demonstrated

- **Lifecycle Node Creation**: Using `createLifecycleNode()`
- **State Callbacks**: Implementing all major transition callbacks
- **Lifecycle Publisher**: Creating and managing lifecycle-aware publishers
- **State Management**: Manual state transitions (`configure()`, `activate()`, etc.)
- **Resource Cleanup**: Proper timer and publisher management
- **Self-Monitoring**: Node subscribing to its own publications

#### Run Command

```bash
node lifecycle-node-example.mjs
```

## Sample Output

When you run the lifecycle example, you'll see output showing the complete state machine progression:

### Expected Output

```
Lifecycle: CONFIGURE
Lifecycle: ACTIVATE
countdown msg: 5
countdown msg: 4
countdown msg: 3
countdown msg: 2
countdown msg: 1
countdown msg: 0
Lifecycle: DEACTIVATE
Lifecycle: SHUTDOWN
```

### Output Explanation

1. **CONFIGURE**: Node enters inactive state, creates publisher and subscriber
2. **ACTIVATE**: Node enters active state, activates publisher, starts timer
3. **countdown msg: X**: Messages published and received showing countdown progress
4. **DEACTIVATE**: Countdown reaches 0, node deactivates publisher and stops timer
5. **SHUTDOWN**: Node cleans up resources and terminates

## Lifecycle Node Implementation Details

### Class Structure

The example uses a class-based approach with these key components:

```javascript
class App {
  constructor() {
    this._node = null; // Lifecycle node instance
    this._publisher = null; // Lifecycle publisher
    this._subscriber = null; // Regular subscriber
    this._timer = null; // Timer for countdown
    this._count = COUNTDOWN; // Current countdown value
  }
}
```

### State Callback Implementation

#### Configure Callback

```javascript
onConfigure() {
  console.log('Lifecycle: CONFIGURE');
  // Create lifecycle publisher
  this._publisher = this._node.createLifecyclePublisher('std_msgs/msg/String', TOPIC);
  // Create regular subscriber
  this._subscriber = this._node.createSubscription('std_msgs/msg/String', TOPIC, callback);
  return rclnodejs.lifecycle.CallbackReturnCode.SUCCESS;
}
```

#### Activate Callback

```javascript
onActivate() {
  console.log('Lifecycle: ACTIVATE');
  // Activate the lifecycle publisher
  this._publisher.activate();
  // Start the countdown timer
  this._timer = this._node.createTimer(BigInt('1000000000'), () => {
    this._publisher.publish(`${this._count--}`);
  });
  return rclnodejs.lifecycle.CallbackReturnCode.SUCCESS;
}
```

#### Deactivate Callback

```javascript
onDeactivate() {
  console.log('Lifecycle: DEACTIVATE');
  // Deactivate publisher
  this._publisher.deactivate();
  // Cancel timer
  if (this._timer) {
    this._timer.cancel();
    this._timer = null;
  }
  return rclnodejs.lifecycle.CallbackReturnCode.SUCCESS;
}
```

#### Shutdown Callback

```javascript
onShutdown(prevState) {
  console.log('Lifecycle: SHUTDOWN');
  // Clean up based on previous state
  if (prevState.id === this._StateInterface.PRIMARY_STATE) {
    this.onDeactivate();
    this._publisher = null;
    this._subscriber = null;
  }
  return rclnodejs.lifecycle.CallbackReturnCode.SUCCESS;
}
```

## Key Lifecycle Concepts

### Lifecycle Publishers

Lifecycle publishers are special publishers that:

- Start in an inactive state when created
- Must be explicitly activated to publish messages
- Can be deactivated to temporarily stop publishing
- Are automatically managed by the lifecycle state machine

```javascript
// Create lifecycle publisher (inactive by default)
const publisher = node.createLifecyclePublisher('std_msgs/msg/String', 'topic');

// Activate publisher (in activate callback)
publisher.activate();

// Deactivate publisher (in deactivate callback)
publisher.deactivate();
```

### State Transitions

Manual state transitions are performed using node methods:

```javascript
// Trigger state transitions
node.configure(); // Unconfigured → Inactive
node.activate(); // Inactive → Active
node.deactivate(); // Active → Inactive
node.shutdown(); // Any state → Finalized
```

### Callback Return Codes

State callbacks must return appropriate codes:

- `CallbackReturnCode.SUCCESS`: Transition succeeded
- `CallbackReturnCode.FAILURE`: Transition failed
- `CallbackReturnCode.ERROR`: Error occurred during transition

## External Lifecycle Management

You can also control lifecycle nodes externally using ROS 2 services:

### Lifecycle Service Interface

Each lifecycle node automatically exposes these services:

- `/node_name/change_state`: Trigger state transitions
- `/node_name/get_state`: Query current state
- `/node_name/get_available_states`: List available states
- `/node_name/get_available_transitions`: List available transitions

### Using ROS 2 CLI Tools

```bash
# Check current state
ros2 lifecycle get /test_node

# List available transitions
ros2 lifecycle list /test_node

# Trigger state transitions
ros2 lifecycle set /test_node configure
ros2 lifecycle set /test_node activate
ros2 lifecycle set /test_node deactivate
ros2 lifecycle set /test_node shutdown
```

## Advanced Lifecycle Patterns

### Error Handling

```javascript
onConfigure() {
  try {
    // Resource initialization
    this._publisher = this._node.createLifecyclePublisher(...);
    return rclnodejs.lifecycle.CallbackReturnCode.SUCCESS;
  } catch (error) {
    console.error('Configuration failed:', error);
    return rclnodejs.lifecycle.CallbackReturnCode.FAILURE;
  }
}
```

### State-Dependent Behavior

```javascript
onActivate() {
  // Only start publishing in active state
  this._timer = this._node.createTimer(interval, () => {
    if (this._node.getCurrentState().label === 'active') {
      this._publisher.publish(data);
    }
  });
  return rclnodejs.lifecycle.CallbackReturnCode.SUCCESS;
}
```

### Resource Monitoring

```javascript
onDeactivate() {
  // Clean up resources
  if (this._timer) {
    this._timer.cancel();
    this._timer = null;
  }

  // Deactivate publishers
  if (this._publisher) {
    this._publisher.deactivate();
  }

  return rclnodejs.lifecycle.CallbackReturnCode.SUCCESS;
}
```

## Use Cases for Lifecycle Nodes

### Hardware Drivers

```javascript
onConfigure() {
  // Initialize hardware connections
  this.initializeHardware();
  return rclnodejs.lifecycle.CallbackReturnCode.SUCCESS;
}

onActivate() {
  // Start hardware communication
  this.startHardwareStreaming();
  return rclnodejs.lifecycle.CallbackReturnCode.SUCCESS;
}
```

### System Services

```javascript
onConfigure() {
  // Load configuration files
  this.loadConfiguration();
  // Setup service endpoints
  this.createServices();
  return rclnodejs.lifecycle.CallbackReturnCode.SUCCESS;
}
```

### Coordinated Systems

Multiple lifecycle nodes can be coordinated using external lifecycle managers:

- System-wide startup sequences
- Graceful shutdown procedures
- Error recovery coordination
- Resource dependency management

## Troubleshooting

### Common Issues

1. **Publisher Not Publishing**:
   - Ensure lifecycle publisher is activated in `onActivate()`
   - Check that node is in active state
   - Verify publisher is created in `onConfigure()`

2. **Timer Not Working**:
   - Create timer in `onActivate()`, not `onConfigure()`
   - Cancel timer in `onDeactivate()`
   - Check timer interval format (BigInt nanoseconds)

3. **State Transition Failures**:
   - Ensure callbacks return appropriate return codes
   - Check for exceptions in callback implementations
   - Verify resource cleanup in deactivate/shutdown

4. **Resource Leaks**:
   - Always cancel timers in `onDeactivate()`
   - Set references to null in `onShutdown()`
   - Properly deactivate lifecycle publishers

### Debugging Tips

- Use `ros2 lifecycle get /node_name` to check current state
- Monitor lifecycle transition services for external control
- Add logging to each callback to trace state transitions
- Verify callback return codes are correct
- Check for proper resource cleanup in each state

## Notes

- Lifecycle nodes require explicit state management
- Publishers created with `createLifecyclePublisher()` start inactive
- Regular subscribers work normally in lifecycle nodes
- State transitions can be triggered internally or externally
- Proper resource cleanup is critical for lifecycle node reliability
- The example demonstrates self-contained lifecycle management
- External lifecycle managers can coordinate multiple lifecycle nodes
- State callbacks must return appropriate success/failure codes
