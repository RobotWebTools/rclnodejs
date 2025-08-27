# ROS 2 Lifecycle Nodes Tutorial

This tutorial explains what lifecycle nodes are, why they're useful, and how to implement them using rclnodejs.

## Table of Contents

- [What are Lifecycle Nodes?](#what-are-lifecycle-nodes)
- [The Lifecycle State Machine](#the-lifecycle-state-machine)
- [Why Use Lifecycle Nodes?](#why-use-lifecycle-nodes)
- [Basic Implementation](#basic-implementation)
- [Complete Example](#complete-example)
- [Lifecycle Publishers](#lifecycle-publishers)
- [External Control](#external-control)
- [Best Practices](#best-practices)
- [Troubleshooting](#troubleshooting)

## What are Lifecycle Nodes?

**Lifecycle nodes** are a special type of ROS 2 node that implement a **managed state machine** for controlled startup, operation, and shutdown sequences. Unlike regular nodes that are immediately active when created, lifecycle nodes go through well-defined states that can be externally controlled.

This provides a standardized way to manage node behavior and resources throughout their operational lifetime, making systems more reliable, controllable, and maintainable.

## The Lifecycle State Machine

Lifecycle nodes implement a state machine with **4 primary states**:

```
┌─────────────┐    configure()    ┌──────────┐    activate()     ┌────────┐
│             │ ───────────────► │          │ ───────────────► │        │
│ Unconfigured│                  │ Inactive │                  │ Active │
│     🔴      │ ◄─────────────── │    🟡    │ ◄─────────────── │   🟢   │
└─────────────┘    cleanup()     └──────────┘   deactivate()   └────────┘
       │                               │                            │
       │                               │                            │
       │              shutdown()       │         shutdown()         │
       └───────────────────────────────┼────────────────────────────┘
                                       │
                                       ▼
                                 ┌──────────┐
                                 │          │
                                 │Finalized │
                                 │    ⚫    │
                                 └──────────┘
```

### States Explained

1. **🔴 Unconfigured** - Initial state, node exists but no resources are allocated
2. **🟡 Inactive** - Node is configured with resources allocated but not actively processing
3. **🟢 Active** - Node is fully operational and actively processing data
4. **⚫ Finalized** - Node is shut down and all resources are cleaned up

### State Transitions

- **configure()** - 🔴 Unconfigured → 🟡 Inactive
- **activate()** - 🟡 Inactive → 🟢 Active
- **deactivate()** - 🟢 Active → 🟡 Inactive
- **cleanup()** - 🟡 Inactive → 🔴 Unconfigured
- **shutdown()** - Any state → ⚫ Finalized

## Why Use Lifecycle Nodes?

Lifecycle nodes are essential for:

### 🚀 Controlled Startup Sequences

```javascript
// Example: Camera node that needs hardware initialization
onConfigure() {
  // Initialize camera hardware
  // Allocate memory buffers
  // Set up image processing pipeline
}

onActivate() {
  // Start camera capture
  // Begin publishing images
}
```

### 🎯 System Orchestration

Start multiple nodes in a specific order:

```bash
# Start all camera nodes first
ros2 lifecycle set /camera_left configure
ros2 lifecycle set /camera_right configure

# Then start processing nodes
ros2 lifecycle set /image_processor configure
ros2 lifecycle set /object_detector configure

# Finally activate everything
ros2 lifecycle set /camera_left activate
ros2 lifecycle set /camera_right activate
ros2 lifecycle set /image_processor activate
ros2 lifecycle set /object_detector activate
```

### 🔄 Error Recovery

Safely restart components without full system restart:

```javascript
// If camera fails, just deactivate and reactivate
onError() {
  this.node.deactivate();
  this.reinitializeCamera();
  this.node.activate();
}
```

### 💾 Resource Management

Only consume resources when actually needed:

```javascript
onActivate() {
  // Only start expensive GPU processing when active
  this.startGPUProcessing();
}

onDeactivate() {
  // Free GPU resources when inactive
  this.stopGPUProcessing();
}
```

### ⚠️ Safety-Critical Systems

Ensure proper initialization before operation:

```javascript
onConfigure() {
  if (!this.safetySystemsCheck()) {
    return rclnodejs.lifecycle.CallbackReturnCode.FAILURE;
  }
  return rclnodejs.lifecycle.CallbackReturnCode.SUCCESS;
}
```

## Basic Implementation

### 1. Create a Lifecycle Node

```javascript
const rclnodejs = require('rclnodejs');

// Initialize ROS 2
await rclnodejs.init();

// Create a lifecycle node
const lifecycleNode = rclnodejs.createLifecycleNode('my_lifecycle_node');
```

### 2. Register Lifecycle Callbacks

```javascript
// Configure: Set up resources
lifecycleNode.registerOnConfigure((prevState) => {
  console.log('🔧 Configuring node...');
  // Initialize resources, create publishers/subscribers
  return rclnodejs.lifecycle.CallbackReturnCode.SUCCESS;
});

// Activate: Start processing
lifecycleNode.registerOnActivate((prevState) => {
  console.log('🟢 Activating node...');
  // Start processing, activate publishers
  return rclnodejs.lifecycle.CallbackReturnCode.SUCCESS;
});

// Deactivate: Stop processing
lifecycleNode.registerOnDeactivate((prevState) => {
  console.log('🟡 Deactivating node...');
  // Stop processing, deactivate publishers
  return rclnodejs.lifecycle.CallbackReturnCode.SUCCESS;
});

// Shutdown: Clean up
lifecycleNode.registerOnShutdown((prevState) => {
  console.log('⚫ Shutting down node...');
  // Clean up resources
  return rclnodejs.lifecycle.CallbackReturnCode.SUCCESS;
});
```

### 3. Start the Node

```javascript
// Start spinning the node
rclnodejs.spin(lifecycleNode);

// Trigger state transitions
lifecycleNode.configure();
lifecycleNode.activate();
```

## Complete Example

Here's a comprehensive example of a robot controller using lifecycle nodes:

```javascript
const rclnodejs = require('rclnodejs');

class LifecycleRobotController {
  constructor() {
    this.node = null;
    this.statusPublisher = null;
    this.commandSubscriber = null;
    this.heartbeatTimer = null;
    this.heartbeatCount = 0;
    this.robotHardware = null;
  }

  async init() {
    await rclnodejs.init();

    // Create lifecycle node
    this.node = rclnodejs.createLifecycleNode('robot_controller');

    // Register lifecycle callbacks
    this.node.registerOnConfigure((prevState) => this.onConfigure(prevState));
    this.node.registerOnActivate((prevState) => this.onActivate(prevState));
    this.node.registerOnDeactivate((prevState) => this.onDeactivate(prevState));
    this.node.registerOnShutdown((prevState) => this.onShutdown(prevState));

    // Start spinning the node
    rclnodejs.spin(this.node);
  }

  onConfigure(prevState) {
    console.log('🔧 Configuring robot controller...');

    try {
      // Initialize robot hardware (simulated)
      this.robotHardware = this.initializeHardware();

      // Create lifecycle publisher for status updates
      this.statusPublisher = this.node.createLifecyclePublisher(
        'std_msgs/msg/String',
        'robot_status'
      );

      // Create subscriber for robot commands
      this.commandSubscriber = this.node.createSubscription(
        'std_msgs/msg/String',
        'robot_commands',
        (msg) => this.handleCommand(msg)
      );

      console.log('✅ Robot controller configured successfully');
      return rclnodejs.lifecycle.CallbackReturnCode.SUCCESS;
    } catch (error) {
      console.error('❌ Failed to configure robot controller:', error);
      return rclnodejs.lifecycle.CallbackReturnCode.FAILURE;
    }
  }

  onActivate(prevState) {
    console.log('🟢 Activating robot controller...');

    try {
      // Activate the lifecycle publisher
      this.statusPublisher.activate();

      // Start robot hardware
      this.robotHardware.start();

      // Start periodic status publishing (1 Hz)
      this.heartbeatTimer = this.node.createTimer(1000000000n, () => {
        const status = {
          data: `Robot active - heartbeat ${this.heartbeatCount++} - ${new Date().toISOString()}`,
        };
        this.statusPublisher.publish(status);
        console.log(`📡 ${status.data}`);
      });

      console.log('✅ Robot controller activated successfully');
      return rclnodejs.lifecycle.CallbackReturnCode.SUCCESS;
    } catch (error) {
      console.error('❌ Failed to activate robot controller:', error);
      return rclnodejs.lifecycle.CallbackReturnCode.FAILURE;
    }
  }

  onDeactivate(prevState) {
    console.log('🟡 Deactivating robot controller...');

    try {
      // Stop robot hardware
      if (this.robotHardware) {
        this.robotHardware.stop();
      }

      // Deactivate publisher (stops publishing)
      if (this.statusPublisher) {
        this.statusPublisher.deactivate();
      }

      // Stop heartbeat timer
      if (this.heartbeatTimer) {
        this.heartbeatTimer.cancel();
        this.heartbeatTimer = null;
      }

      console.log('✅ Robot controller deactivated successfully');
      return rclnodejs.lifecycle.CallbackReturnCode.SUCCESS;
    } catch (error) {
      console.error('❌ Failed to deactivate robot controller:', error);
      return rclnodejs.lifecycle.CallbackReturnCode.ERROR;
    }
  }

  onShutdown(prevState) {
    console.log('⚫ Shutting down robot controller...');

    try {
      // Clean up hardware
      if (this.robotHardware) {
        this.robotHardware.shutdown();
        this.robotHardware = null;
      }

      // Clean up timer
      if (this.heartbeatTimer) {
        this.heartbeatTimer.cancel();
        this.heartbeatTimer = null;
      }

      // Clear references
      this.statusPublisher = null;
      this.commandSubscriber = null;

      console.log('✅ Robot controller shut down successfully');
      return rclnodejs.lifecycle.CallbackReturnCode.SUCCESS;
    } catch (error) {
      console.error('❌ Failed to shutdown robot controller:', error);
      return rclnodejs.lifecycle.CallbackReturnCode.ERROR;
    }
  }

  // Handle incoming robot commands
  handleCommand(msg) {
    console.log(`🎮 Received command: ${msg.data}`);

    switch (msg.data.toLowerCase()) {
      case 'stop':
        console.log('🛑 Stop command received, shutting down...');
        this.shutdown();
        break;
      case 'pause':
        console.log('⏸️ Pause command received, deactivating...');
        this.node.deactivate();
        break;
      case 'resume':
        console.log('▶️ Resume command received, activating...');
        this.node.activate();
        break;
      default:
        if (this.robotHardware) {
          this.robotHardware.executeCommand(msg.data);
        }
    }
  }

  // Simulate hardware initialization
  initializeHardware() {
    return {
      start: () => console.log('🤖 Robot hardware started'),
      stop: () => console.log('🤖 Robot hardware stopped'),
      shutdown: () => console.log('🤖 Robot hardware shutdown'),
      executeCommand: (cmd) => console.log(`🤖 Executing: ${cmd}`),
    };
  }

  // Public methods to control the lifecycle
  async start() {
    console.log('🚀 Starting robot controller lifecycle...');
    this.node.configure();
    this.node.activate();
  }

  shutdown() {
    console.log('🔚 Initiating shutdown sequence...');
    this.node.deactivate();
    this.node.shutdown();
    rclnodejs.shutdown();
    process.exit(0);
  }
}

// Usage example
async function main() {
  const controller = new LifecycleRobotController();

  try {
    await controller.init();
    await controller.start();

    // Handle graceful shutdown
    process.on('SIGINT', () => {
      console.log('\n🛑 Received SIGINT, shutting down gracefully...');
      controller.shutdown();
    });

    console.log(
      '🎯 Robot controller is running. Send commands to /robot_commands topic.'
    );
    console.log(
      '💡 Available commands: stop, pause, resume, or any custom command'
    );
    console.log('📊 Status updates published to /robot_status topic');
  } catch (error) {
    console.error('💥 Failed to start robot controller:', error);
    process.exit(1);
  }
}

// Start the application
main().catch(console.error);
```

## Lifecycle Publishers

One key feature is **Lifecycle Publishers** that only publish when the node is active:

```javascript
// Create a lifecycle publisher
const lifecyclePublisher = node.createLifecyclePublisher(
  'sensor_msgs/msg/Image',
  'camera/image'
);

// In onActivate callback - start publishing
onActivate() {
  lifecyclePublisher.activate();
  // Now publisher will actually send messages
}

// In onDeactivate callback - stop publishing
onDeactivate() {
  lifecyclePublisher.deactivate();
  // Publisher stops sending messages but stays configured
}

// Usage - publish() calls only work when publisher is active
lifecyclePublisher.publish(imageMessage); // Only works when active
```

## External Control

### Using ROS 2 CLI

Lifecycle nodes can be controlled externally using ROS 2 command-line tools:

```bash
# Check current state
ros2 lifecycle get /robot_controller

# List available transitions
ros2 lifecycle list /robot_controller

# Trigger transitions
ros2 lifecycle set /robot_controller configure
ros2 lifecycle set /robot_controller activate
ros2 lifecycle set /robot_controller deactivate
ros2 lifecycle set /robot_controller shutdown
```

### Using ROS 2 Services

You can also control lifecycle nodes programmatically using services:

```javascript
// Create a client to control another lifecycle node
const lifecycleClient = node.createClient(
  'lifecycle_msgs/srv/ChangeState',
  '/other_node/change_state'
);

// Trigger a state change
const request = {
  transition: {
    id: 1, // Configure transition
    label: 'configure',
  },
};

const response = await lifecycleClient.sendRequest(request);
console.log('Transition result:', response.success);
```

### System Management Script

Here's an example script to orchestrate multiple lifecycle nodes:

```javascript
// system-manager.js
const rclnodejs = require('rclnodejs');

class SystemManager {
  constructor() {
    this.nodes = [
      'camera_left',
      'camera_right',
      'image_processor',
      'robot_controller',
    ];
  }

  async init() {
    await rclnodejs.init();
    this.node = rclnodejs.create_node('system_manager');
  }

  async startupSequence() {
    console.log('🚀 Starting system startup sequence...');

    // Phase 1: Configure all nodes
    for (const nodeName of this.nodes) {
      await this.transitionNode(nodeName, 'configure');
    }

    // Phase 2: Activate in specific order
    const activationOrder = [
      'camera_left',
      'camera_right',
      'image_processor',
      'robot_controller',
    ];
    for (const nodeName of activationOrder) {
      await this.transitionNode(nodeName, 'activate');
      await this.sleep(1000); // Wait 1 second between activations
    }

    console.log('✅ System startup complete!');
  }

  async transitionNode(nodeName, transition) {
    // Implementation would call lifecycle service
    console.log(`🔄 ${transition} ${nodeName}`);
  }

  sleep(ms) {
    return new Promise((resolve) => setTimeout(resolve, ms));
  }
}
```

## Best Practices

### 1. Return Codes

Always return appropriate codes from lifecycle callbacks:

```javascript
// Available return codes
rclnodejs.lifecycle.CallbackReturnCode.SUCCESS  // Transition successful
rclnodejs.lifecycle.CallbackReturnCode.FAILURE  // Transition failed
rclnodejs.lifecycle.CallbackReturnCode.ERROR    // Error occurred

onConfigure() {
  try {
    this.initializeResources();
    return rclnodejs.lifecycle.CallbackReturnCode.SUCCESS;
  } catch (error) {
    console.error('Configuration failed:', error);
    return rclnodejs.lifecycle.CallbackReturnCode.FAILURE;
  }
}
```

### 2. Resource Management

Follow the resource allocation pattern:

```javascript
onConfigure() {
  // ✅ Allocate resources (create publishers, open files, etc.)
  this.publisher = this.node.createLifecyclePublisher(...);
  this.fileHandle = fs.openSync('data.log', 'w');
}

onActivate() {
  // ✅ Start using resources (activate publishers, start timers)
  this.publisher.activate();
  this.timer = this.node.createTimer(...);
}

onDeactivate() {
  // ✅ Stop using resources (deactivate publishers, stop timers)
  this.publisher.deactivate();
  this.timer.cancel();
}

onShutdown() {
  // ✅ Free resources (close files, cleanup)
  fs.closeSync(this.fileHandle);
  this.publisher = null;
}
```

### 3. Error Handling

Implement robust error handling:

```javascript
onActivate() {
  try {
    this.hardware.start();
    this.publisher.activate();
    return rclnodejs.lifecycle.CallbackReturnCode.SUCCESS;
  } catch (error) {
    console.error('Activation failed:', error);
    // Try to cleanup partial state
    this.safeCleanup();
    return rclnodejs.lifecycle.CallbackReturnCode.FAILURE;
  }
}

safeCleanup() {
  try {
    if (this.hardware) this.hardware.stop();
    if (this.publisher) this.publisher.deactivate();
  } catch (cleanupError) {
    console.error('Cleanup failed:', cleanupError);
  }
}
```

### 4. State Validation

Validate state before operations:

```javascript
publishData(data) {
  // Only publish if node is active
  if (this.node.getCurrentState().label === 'active') {
    this.publisher.publish(data);
  } else {
    console.warn('Cannot publish: node is not active');
  }
}
```

## Troubleshooting

### Common Issues

1. **Callback Returns Wrong Code**

```javascript
// ❌ Wrong - forgetting to return
onConfigure() {
  this.setup();
  // Missing return statement!
}

// ✅ Correct
onConfigure() {
  this.setup();
  return rclnodejs.lifecycle.CallbackReturnCode.SUCCESS;
}
```

2. **Resource Leaks**

```javascript
// ❌ Wrong - not cleaning up in shutdown
onShutdown() {
  return rclnodejs.lifecycle.CallbackReturnCode.SUCCESS;
  // Forgot to cleanup timer, file handles, etc.
}

// ✅ Correct
onShutdown() {
  if (this.timer) this.timer.cancel();
  if (this.fileHandle) fs.closeSync(this.fileHandle);
  return rclnodejs.lifecycle.CallbackReturnCode.SUCCESS;
}
```

3. **Wrong State Transitions**

```bash
# ❌ Wrong - cannot go directly from unconfigured to active
ros2 lifecycle set /my_node activate  # Will fail

# ✅ Correct - must configure first
ros2 lifecycle set /my_node configure
ros2 lifecycle set /my_node activate
```

### Debugging Tips

1. **Check Current State**

```bash
ros2 lifecycle get /my_node
```

2. **Monitor State Changes**

```bash
ros2 topic echo /my_node/transition_event
```

3. **Enable Debug Logging**

```javascript
// Add logging to callbacks
onConfigure() {
  console.log('Starting configuration...');
  // ... configuration code ...
  console.log('Configuration complete');
  return rclnodejs.lifecycle.CallbackReturnCode.SUCCESS;
}
```

### Performance Considerations

- **Quick Transitions**: Keep lifecycle callbacks fast - avoid long-running operations
- **Resource Pooling**: Reuse resources between deactivate/activate cycles when possible
- **Async Operations**: Handle async operations carefully in callbacks
- **Memory Management**: Properly cleanup to avoid memory leaks

## Conclusion

Lifecycle nodes provide a powerful framework for building robust, manageable ROS 2 systems. They enable:

- **🎯 Predictable startup and shutdown sequences**
- **🔧 Better resource management**
- **🛡️ Improved error handling and recovery**
- **📊 System-level orchestration and monitoring**

By following the patterns and best practices in this tutorial, you can build more reliable and maintainable robotics applications with rclnodejs.

For more examples, see the [lifecycle example](../example/lifecycle/) in the rclnodejs repository.
