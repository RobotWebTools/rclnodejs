# ROS 2 Actions Examples

This directory contains examples demonstrating ROS 2 action-based communication using rclnodejs. Actions provide a more complex communication pattern than topics or services, designed for long-running tasks that require feedback and can be canceled.

## Overview

ROS 2 actions are built on top of topics and services to provide:

- **Goal**: A request to perform some long-running task
- **Feedback**: Periodic updates on progress during task execution
- **Result**: The final outcome when the task completes
- **Cancellation**: Ability to stop the task before completion

Actions are ideal for:

- Navigation tasks (moving a robot to a location)
- Long-running computations
- Tasks that need progress updates
- Operations that might need to be canceled

## Action Examples

### Action Client Examples

The `action_client/` directory contains examples of nodes that send goals to action servers:

#### 1. Basic Action Client (`action-client-example.cjs`)

**Purpose**: Demonstrates basic action client functionality with the Fibonacci action.

- **Action Type**: `test_msgs/action/Fibonacci`
- **Action Name**: `fibonacci`
- **Functionality**:
  - Sends a goal to compute Fibonacci sequence up to order 10
  - Receives and logs feedback during execution
  - Waits for final result and logs success/failure
  - Automatically shuts down after completion
- **Features**:
  - Server availability checking with `waitForServer()`
  - Goal acceptance verification
  - Feedback handling during execution
  - Result processing with status checking
- **Run Command**: `node action_client/action-client-example.cjs`

#### 2. Action Client with Cancellation (`action-client-cancel-example.cjs`)

**Purpose**: Demonstrates how to cancel an action goal during execution.

- **Action Type**: `test_msgs/action/Fibonacci`
- **Action Name**: `fibonacci`
- **Functionality**:
  - Sends a goal to compute Fibonacci sequence up to order 10
  - Receives feedback for 2 seconds
  - Automatically cancels the goal after timer expires
  - Logs cancellation success/failure
- **Features**:
  - Timer-based cancellation mechanism
  - Goal cancellation with `cancelGoal()`
  - Cancellation response verification
  - Cleanup and shutdown handling
- **Run Command**: `node action_client/action-client-cancel-example.cjs`

#### 3. Action Client Validation (`action-client-validation-example.cjs`)

**Purpose**: Demonstrates goal validation features for action clients.

- **Action Type**: `action_tutorials_interfaces/action/Fibonacci`
- **Action Name**: `fibonacci`
- **Functionality**:
  - Schema introspection for action goal types
  - Client-level validation with `validateGoals: true` option
  - Per-goal validation override with `{ validate: true/false }`
  - Strict mode validation for detecting unknown fields
  - Reusable goal validators with `createMessageValidator()`
  - Error handling with `MessageValidationError`
- **Features**:
  - **Goal Validation**: Catch invalid goals before sending to action server
  - **Schema Introspection**: Use `getMessageSchema()` to inspect goal structure
  - **Dynamic Toggle**: Enable/disable validation with `willValidateGoal` property
  - **Detailed Errors**: Field-level validation issues with expected vs received types
  - **Strict Mode**: Detect extra fields that don't belong in the goal
  - **Reusable Validators**: Create validators for repeated goal validation
- **Run Command**: `node action_client/action-client-validation-example.cjs`
- **Note**: Standalone example - demonstrates validation errors without requiring a running action server

### Action Server Examples

The `action_server/` directory contains examples of nodes that provide action services:

#### 1. Basic Action Server (`action-server-example.cjs`)

**Purpose**: Demonstrates basic action server implementation for computing Fibonacci sequences.

- **Action Type**: `test_msgs/action/Fibonacci`
- **Action Name**: `fibonacci`
- **Functionality**:
  - Accepts goals to compute Fibonacci sequences
  - Publishes feedback with incremental sequence updates
  - Returns complete sequence as result
  - Supports goal cancellation during execution
- **Features**:
  - Goal acceptance callback (`goalCallback`)
  - Execution callback with feedback publishing
  - Cancellation handling (`cancelCallback`)
  - Progress updates every second
- **Run Command**: `node action_server/action-server-example.cjs`

#### 2. Deferred Execution Server (`action-server-defer-example.cjs`)

**Purpose**: Shows how to defer goal execution using timers and handle accepted callbacks.

- **Action Type**: `test_msgs/action/Fibonacci`
- **Action Name**: `fibonacci`
- **Functionality**:
  - Accepts goals immediately but defers execution for 3 seconds
  - Uses timer to delay goal execution start
  - Demonstrates deferred execution pattern
  - Same Fibonacci computation as basic server
- **Features**:
  - Goal acceptance with deferred execution
  - Handle accepted callback (`handleAcceptedCallback`)
  - Timer-based execution control
  - Manual goal execution triggering
- **Run Command**: `node action_server/action-server-defer-example.cjs`

#### 3. Single Goal Server (`action-server-single-goal-example.cjs`)

**Purpose**: Demonstrates a server that only allows one active goal at a time.

- **Action Type**: `test_msgs/action/Fibonacci`
- **Action Name**: `fibonacci`
- **Functionality**:
  - Accepts new goals but aborts any currently active goal
  - Ensures only one goal executes at a time
  - Tracks active goal state
  - Same Fibonacci computation with single-goal constraint
- **Features**:
  - Single goal enforcement
  - Automatic abortion of previous goals
  - Goal state tracking (`isActive`)
  - Handle accepted callback for goal management
- **Run Command**: `node action_server/action-server-single-goal-example.cjs`

## How to Run the Examples

### Prerequisites

1. Ensure ROS 2 is installed and sourced
2. Navigate to the `example/actions` directory

### Running Complete Action Examples

#### Basic Action Communication

1. **Start the Action Server**: In one terminal, run:

   ```bash
   cd example/actions
   node action_server/action-server-example.cjs
   ```

   You should see:

   ```
   [INFO] [action_server_example_node]: Action server started
   ```

2. **Start the Action Client**: In another terminal, run:

   ```bash
   cd example/actions
   node action_client/action-client-example.cjs
   ```

3. **Expected Output**:

   **Action Server Terminal**:

   ```
   [INFO] [action_server_example_node]: Received goal request
   [INFO] [action_server_example_node]: Executing goal...
   [INFO] [action_server_example_node]: Publishing feedback: 0,1
   [INFO] [action_server_example_node]: Publishing feedback: 0,1,1
   [INFO] [action_server_example_node]: Publishing feedback: 0,1,1,2
   ...
   [INFO] [action_server_example_node]: Returning result: 0,1,1,2,3,5,8,13,21,34,55
   ```

   **Action Client Terminal**:

   ```
   [INFO] [action_client_example_node]: Waiting for action server...
   [INFO] [action_client_example_node]: Sending goal request...
   [INFO] [action_client_example_node]: Goal accepted
   [INFO] [action_client_example_node]: Received feedback: 0,1
   [INFO] [action_client_example_node]: Received feedback: 0,1,1
   ...
   [INFO] [action_client_example_node]: Goal succeeded with result: 0,1,1,2,3,5,8,13,21,34,55
   ```

#### Action Cancellation Example

1. **Start Server**: Run any action server example
2. **Start Cancellation Client**:
   ```bash
   node action_client/action-client-cancel-example.cjs
   ```
3. **Expected Behavior**: Client sends goal, receives feedback for 2 seconds, then cancels

#### Specialized Server Examples

- **Deferred Execution**: Use `action-server-defer-example.cjs` to see 3-second execution delay
- **Single Goal**: Use `action-server-single-goal-example.cjs` to test goal abortion behavior

## Action Components Explained

### Action Message Structure

The `test_msgs/action/Fibonacci` action consists of:

```
# Goal
int32 order
---
# Result
int32[] sequence
---
# Feedback
int32[] sequence
```

### Key Concepts Demonstrated

#### Action Client Concepts

- **Goal Sending**: Using `sendGoal()` with feedback callbacks
- **Server Discovery**: Waiting for action servers with `waitForServer()`
- **Goal Status**: Checking acceptance and completion status
- **Feedback Handling**: Processing incremental updates during execution
- **Result Processing**: Handling final results and status
- **Goal Cancellation**: Canceling active goals with `cancelGoal()`
- **Goal Validation**: Pre-send validation with `validateGoals` option and `MessageValidationError`
- **Schema Introspection**: Programmatic access to action goal schemas

#### Action Server Concepts

- **Goal Callbacks**: Accepting or rejecting incoming goals
- **Execution Callbacks**: Implementing the actual action logic
- **Feedback Publishing**: Sending progress updates to clients
- **Result Handling**: Returning final results upon completion
- **Cancellation Support**: Responding to cancellation requests
- **Goal State Management**: Tracking active goals and their status

#### Advanced Patterns

- **Deferred Execution**: Accepting goals but delaying execution start
- **Single Goal Servers**: Limiting concurrent goal execution
- **Goal Abortion**: Terminating active goals when new ones arrive
- **Timer Integration**: Using ROS timers for delayed operations

## Programming Patterns

### Class-Based Architecture

All examples use ES6 classes to encapsulate action functionality:

- Clean separation of client/server logic
- Proper callback binding with `bind(this)`
- State management through instance variables

### Asynchronous Operations

- **Async/Await**: Modern JavaScript patterns for goal handling
- **Promises**: Integration with ROS action lifecycle
- **Callbacks**: Feedback and result processing

### Resource Management

- **Timer Cleanup**: Proper timer cancellation
- **Goal Tracking**: Maintaining references to active goals
- **Shutdown Handling**: Clean node shutdown after completion

## Troubleshooting

### Common Issues

1. **Action Server Not Available**:
   - Ensure action server is running before starting client
   - Check that both use the same action name (`fibonacci`)
   - Verify action type matches (`test_msgs/action/Fibonacci`)

2. **Goal Not Accepted**:
   - Check server's `goalCallback` return value
   - Verify goal message structure is correct
   - Ensure server is properly initialized

3. **Missing Feedback**:
   - Confirm feedback callback is properly bound
   - Check server's `publishFeedback()` calls
   - Verify feedback message structure

4. **Cancellation Issues**:
   - Ensure server implements `cancelCallback`
   - Check `isCancelRequested` in execution loop
   - Verify proper `goalHandle.canceled()` calls

### Debugging Tips

- Use `ros2 action list` to see available actions
- Use `ros2 action info <action_name>` to check action details
- Use `ros2 action send_goal <action_name> <action_type> <goal>` to test from command line
- Monitor action topics: `/_action/status`, `/_action/feedback`, `/_action/result`

## Notes

- All examples use the Fibonacci sequence computation as a representative long-running task
- Action servers run continuously until manually terminated (Ctrl+C)
- Action clients typically complete one goal cycle then exit
- Goals are processed with 1-second intervals to demonstrate feedback clearly
- Cancellation examples use timers to simulate real-world cancellation scenarios
- Single goal servers demonstrate resource management for concurrent requests
