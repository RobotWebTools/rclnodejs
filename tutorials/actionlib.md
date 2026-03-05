# ROS 2 Actions Tutorial

This tutorial explains how to use ROS 2 actions in rclnodejs for implementing long-running, cancellable tasks with feedback.

## Table of Contents

- [What are ROS 2 Actions?](#what-are-ros-2-actions)
- [Action Components](#action-components)
- [Basic Implementation](#basic-implementation)
- [Action Server Example](#action-server-example)
- [Action Client Example](#action-client-example)
- [Advanced Features](#advanced-features)
- [Best Practices](#best-practices)
- [Running the Examples](#running-the-examples)

## What are ROS 2 Actions?

**ROS 2 Actions** are a communication pattern designed for **long-running, preemptable tasks** that provide periodic feedback. Unlike simple request-response services, actions allow clients to:

- 📤 **Send goals** to request task execution
- 📊 **Receive feedback** during task execution
- 🎯 **Get results** when tasks complete
- ❌ **Cancel goals** before completion

Actions are built on top of topics and services, providing a higher-level abstraction for complex interactions.

## Action Components

An action consists of three message types:

### 1. Goal Message

Defines the request parameters for the task to be performed.

```javascript
// Example: Fibonacci.Goal
{
  order: 10; // Compute Fibonacci sequence up to order 10
}
```

### 2. Feedback Message

Provides periodic updates during task execution.

```javascript
// Example: Fibonacci.Feedback
{
  sequence: [0, 1, 1, 2, 3, 5, 8]; // Current progress
}
```

### 3. Result Message

Contains the final outcome when the task completes.

```javascript
// Example: Fibonacci.Result
{
  sequence: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55]; // Final result
}
```

## Basic Implementation

### Action Types in rclnodejs

rclnodejs provides two main classes for action communication:

- **`ActionServer`** - Handles incoming goals and executes tasks
- **`ActionClient`** - Sends goals and receives feedback/results

### Import Action Messages

```javascript
const rclnodejs = require('rclnodejs');

// Import action message types
const Fibonacci = rclnodejs.require('test_msgs/action/Fibonacci');
```

## Action Server Example

An action server receives goals, executes tasks, provides feedback, and returns results.

### Basic Action Server Structure

```javascript
const rclnodejs = require('rclnodejs');
const Fibonacci = rclnodejs.require('test_msgs/action/Fibonacci');

class FibonacciActionServer {
  constructor(node) {
    this._node = node;

    // Create action server
    this._actionServer = new rclnodejs.ActionServer(
      node, // ROS 2 node
      'test_msgs/action/Fibonacci', // Action type
      'fibonacci', // Action name
      this.executeCallback.bind(this), // Execute callback
      this.goalCallback.bind(this), // Goal callback (optional)
      null, // Handle accepted callback (optional)
      this.cancelCallback.bind(this) // Cancel callback (optional)
    );
  }

  // Main execution logic
  async executeCallback(goalHandle) {
    this._node.getLogger().info('Executing goal...');

    const feedbackMessage = new Fibonacci.Feedback();
    const sequence = [0, 1];

    // Execute the task with feedback
    for (let i = 1; i < goalHandle.request.order; i++) {
      // Check if goal was canceled
      if (goalHandle.isCancelRequested) {
        goalHandle.canceled();
        this._node.getLogger().info('Goal canceled');
        return new Fibonacci.Result();
      }

      // Update sequence
      sequence.push(sequence[i] + sequence[i - 1]);
      feedbackMessage.sequence = sequence;

      // Publish feedback
      goalHandle.publishFeedback(feedbackMessage);
      this._node
        .getLogger()
        .info(`Publishing feedback: ${feedbackMessage.sequence}`);

      // Simulate work
      await new Promise((resolve) => setTimeout(resolve, 1000));
    }

    // Mark goal as succeeded
    goalHandle.succeed();

    // Return final result
    const result = new Fibonacci.Result();
    result.sequence = sequence;
    this._node.getLogger().info(`Returning result: ${result.sequence}`);

    return result;
  }

  // Called when new goal is received
  goalCallback(goal) {
    this._node.getLogger().info('Received goal request');
    // Accept or reject the goal
    return rclnodejs.GoalResponse.ACCEPT; // or REJECT
  }

  // Called when cancel is requested
  cancelCallback(goalHandle) {
    this._node.getLogger().info('Received cancel request');
    // Accept or reject the cancellation
    return rclnodejs.CancelResponse.ACCEPT; // or REJECT
  }
}

// Initialize and run server
rclnodejs
  .init()
  .then(() => {
    const node = rclnodejs.createNode('fibonacci_action_server');
    new FibonacciActionServer(node);
    rclnodejs.spin(node);
  })
  .catch(console.error);
```

## Action Client Example

An action client sends goals, receives feedback, and handles results.

### Basic Action Client Structure

```javascript
const rclnodejs = require('rclnodejs');
const Fibonacci = rclnodejs.require('test_msgs/action/Fibonacci');

class FibonacciActionClient {
  constructor(node) {
    this._node = node;

    // Create action client
    this._actionClient = new rclnodejs.ActionClient(
      node, // ROS 2 node
      'test_msgs/action/Fibonacci', // Action type
      'fibonacci' // Action name
    );
  }

  async sendGoal() {
    // Wait for action server to be available
    this._node.getLogger().info('Waiting for action server...');
    await this._actionClient.waitForServer();

    // Create goal message
    const goal = new Fibonacci.Goal();
    goal.order = 10;

    this._node.getLogger().info('Sending goal request...');

    // Send goal with feedback callback
    const goalHandle = await this._actionClient.sendGoal(goal, (feedback) =>
      this.feedbackCallback(feedback)
    );

    // Check if goal was accepted
    if (!goalHandle.isAccepted()) {
      this._node.getLogger().info('Goal rejected');
      return;
    }

    this._node.getLogger().info('Goal accepted');

    // Wait for result
    const result = await goalHandle.getResult();

    // Handle final result
    if (goalHandle.isSucceeded()) {
      this._node
        .getLogger()
        .info(`Goal succeeded with result: ${result.sequence}`);
    } else {
      this._node
        .getLogger()
        .info(`Goal failed with status: ${goalHandle.status}`);
    }

    rclnodejs.shutdown();
  }

  // Handle feedback during execution
  feedbackCallback(feedback) {
    this._node.getLogger().info(`Received feedback: ${feedback.sequence}`);
  }
}

// Initialize and run client
rclnodejs
  .init()
  .then(async () => {
    const node = rclnodejs.createNode('fibonacci_action_client');
    const client = new FibonacciActionClient(node);

    rclnodejs.spin(node);
    await client.sendGoal();
  })
  .catch(console.error);
```

## Advanced Features

### Goal Cancellation

Clients can cancel goals during execution:

```javascript
class CancelableActionClient {
  async sendCancelableGoal() {
    const goal = new Fibonacci.Goal();
    goal.order = 20;

    const goalHandle = await this._actionClient.sendGoal(goal, (feedback) => {
      console.log(`Feedback: ${feedback.sequence}`);
    });

    if (goalHandle.isAccepted()) {
      // Cancel after 3 seconds
      setTimeout(async () => {
        console.log('Canceling goal...');
        const cancelResponse = await goalHandle.cancelGoal();

        if (cancelResponse.goals_canceling.length > 0) {
          console.log('Goal cancellation accepted');
        }
      }, 3000);

      const result = await goalHandle.getResult();
      console.log(`Final status: ${goalHandle.status}`);
    }
  }
}
```

### Multiple Goals

Action servers can handle multiple concurrent goals:

```javascript
class MultiGoalActionServer {
  constructor(node) {
    this._node = node;
    this._activeGoals = new Map();

    this._actionServer = new rclnodejs.ActionServer(
      node,
      'test_msgs/action/Fibonacci',
      'fibonacci',
      this.executeCallback.bind(this),
      this.goalCallback.bind(this)
    );
  }

  goalCallback(goal) {
    // Accept up to 3 concurrent goals
    if (this._activeGoals.size >= 3) {
      this._node.getLogger().info('Too many active goals, rejecting');
      return rclnodejs.GoalResponse.REJECT;
    }

    this._node.getLogger().info(`Accepting goal with order=${goal.order}`);
    return rclnodejs.GoalResponse.ACCEPT;
  }

  async executeCallback(goalHandle) {
    this._activeGoals.set(goalHandle.goalId, goalHandle);
    try {
      // Execute goal logic...
      const result = await this.computeFibonacci(goalHandle);

      goalHandle.succeed();
      return result;
    } finally {
      // Clean up when done
      this._activeGoals.delete(goalHandle.goalId);
    }
  }
}
```

### Goal Status Monitoring

Monitor goal status changes:

```javascript
const goalHandle = await this._actionClient.sendGoal(goal);

// Check goal status
if (goalHandle.isAccepted()) {
  console.log('Goal accepted');
} else {
  console.log('Goal rejected');
}

const result = await goalHandle.getResult();

// Check final status
if (goalHandle.isSucceeded()) {
  console.log('Goal succeeded');
} else if (goalHandle.isCanceled()) {
  console.log('Goal was canceled');
} else if (goalHandle.isAborted()) {
  console.log('Goal was aborted');
}
```

## Best Practices

### 1. Error Handling

Always implement proper error handling:

```javascript
async executeCallback(goalHandle) {
  try {
    // Task execution logic
    const result = await this.performTask(goalHandle.request);
    goalHandle.succeed();
    return result;
  } catch (error) {
    this._node.getLogger().error(`Task failed: ${error.message}`);
    goalHandle.abort();
    return new TaskResult();
  }
}
```

### 2. Responsive Cancellation

Check for cancellation requests regularly:

```javascript
async executeCallback(goalHandle) {
  for (let i = 0; i < longRunningTask.steps; i++) {
    // Check for cancellation
    if (goalHandle.isCancelRequested) {
      goalHandle.canceled();
      return new TaskResult();
    }

    // Perform one step
    await this.performStep(i);

    // Provide feedback
    const feedback = new TaskFeedback();
    feedback.progress = (i + 1) / longRunningTask.steps;
    goalHandle.publishFeedback(feedback);
  }

  goalHandle.succeed();
  return result;
}
```

### 3. Server Availability

Always wait for server availability:

```javascript
async sendGoal() {
  try {
    // Wait for server with timeout (5 seconds)
    this._node.getLogger().info('Waiting for action server...');
    await this._actionClient.waitForServer(5000);

    this._node.getLogger().info('Action server available');

    // Proceed with goal sending
    const goal = new Fibonacci.Goal();
    goal.order = 10;
    const goalHandle = await this._actionClient.sendGoal(goal);
  } catch (error) {
    this._node.getLogger().error('Action server not available within timeout');
    return;
  }
}
```

### 4. Resource Cleanup

Properly clean up resources:

```javascript
class ActionNode {
  constructor() {
    this._node = rclnodejs.createNode('action_node');
    this._client = new rclnodejs.ActionClient(
      this._node,
      'MyAction',
      'my_action'
    );

    // Handle shutdown
    process.on('SIGINT', () => this.shutdown());
  }

  shutdown() {
    this._node.getLogger().info('Shutting down...');
    rclnodejs.shutdown();
  }
}
```

## Running the Examples

The rclnodejs repository includes complete action examples in the `example/actions/` directory.

### Run Action Server

```bash
# Terminal 1 - Start the action server
cd /path/to/rclnodejs
node example/actions/action_server/action-server-example.js
```

### Run Action Client

```bash
# Terminal 2 - Run the action client
cd /path/to/rclnodejs
node example/actions/action_client/action-client-example.js
```

### Expected Output

**Action Server Output:**

```
[INFO] [action_server_example_node]: Received goal request
[INFO] [action_server_example_node]: Executing goal...
[INFO] [action_server_example_node]: Publishing feedback: 0,1
[INFO] [action_server_example_node]: Publishing feedback: 0,1,1
[INFO] [action_server_example_node]: Publishing feedback: 0,1,1,2
...
[INFO] [action_server_example_node]: Returning result: 0,1,1,2,3,5,8,13,21,34,55
```

**Action Client Output:**

```
[INFO] [action_client_example_node]: Waiting for action server...
[INFO] [action_client_example_node]: Sending goal request...
[INFO] [action_client_example_node]: Goal accepted
[INFO] [action_client_example_node]: Received feedback: 0,1
[INFO] [action_client_example_node]: Received feedback: 0,1,1
[INFO] [action_client_example_node]: Received feedback: 0,1,1,2
...
[INFO] [action_client_example_node]: Goal succeeded with result: 0,1,1,2,3,5,8,13,21,34,55
```

### Additional Examples

Explore more examples in the `example/actions/` directory:

- **`action-client-cancel-example.js`** - Demonstrates goal cancellation
- **`action-server-defer-example.js`** - Shows deferred goal acceptance
- **`action-server-single-goal-example.js`** - Single goal handling pattern

## Action Message Creation

For custom actions, create `.action` files with the following structure:

```
# Goal definition
int32 order
---
# Result definition
int32[] sequence
---
# Feedback definition
int32[] partial_sequence
```

Generate JavaScript interfaces using:

```bash
npx generate-ros-messages
```

This tutorial provides a comprehensive guide to using ROS 2 actions with rclnodejs. Actions are powerful tools for implementing complex, long-running robotics tasks with proper feedback and cancellation support.
