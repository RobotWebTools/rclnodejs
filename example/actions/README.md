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

The `action_client/` directory contains native ROS 2 clients and an HTTP client that send goals to action servers:

#### 1. Basic Action Client (`action-client-example.mjs`)

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
- **Run Command**: `node action_client/action-client-example.mjs`

#### 2. Action Client with Cancellation (`action-client-cancel-example.mjs`)

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
- **Run Command**: `node action_client/action-client-cancel-example.mjs`

#### 3. Action Client Validation (`action-client-validation-example.mjs`)

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
- **Run Command**: `node action_client/action-client-validation-example.mjs`
- **Note**: Standalone example - demonstrates validation errors without requiring a running action server

#### 4. HTTP Action Client (`http-action-client-example.mjs`)

Uses the `rclnodejs/web` SDK to send a Fibonacci goal over HTTP, receive SSE feedback, and print the result and terminal status. The client does not create a ROS node and always closes its HTTP stream. It exits with a nonzero status for request/stream errors or a goal that does not succeed.

See [HTTP Actions over SSE](#http-actions-over-sse) for the server, runtime, and client commands.

### Action Server Examples

The `action_server/` directory contains examples of nodes that provide action services:

#### 1. Basic Action Server (`action-server-example.mjs`)

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
- **Run Command**: `node action_server/action-server-example.mjs`

#### 2. Deferred Execution Server (`action-server-defer-example.mjs`)

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
- **Run Command**: `node action_server/action-server-defer-example.mjs`

#### 3. Single Goal Server (`action-server-single-goal-example.mjs`)

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
- **Run Command**: `node action_server/action-server-single-goal-example.mjs`

## How to Run the Examples

### Prerequisites

1. Ensure ROS 2 is installed and sourced
2. Navigate to the `example/actions` directory

### Running Complete Action Examples

#### Basic Action Communication

1. **Start the Action Server**: In one terminal, run:

   ```bash
   cd example/actions
   node action_server/action-server-example.mjs
   ```

   You should see:

   ```
   [INFO] [action_server_example_node]: Action server started
   ```

2. **Start the Action Client**: In another terminal, run:

   ```bash
   cd example/actions
   node action_client/action-client-example.mjs
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
   node action_client/action-client-cancel-example.mjs
   ```
3. **Expected Behavior**: Client sends goal, receives feedback for 2 seconds, then cancels

#### Specialized Server Examples

- **Deferred Execution**: Use `action-server-defer-example.mjs` to see 3-second execution delay
- **Single Goal**: Use `action-server-single-goal-example.mjs` to test goal abortion behavior

## HTTP Actions over SSE

Run the following commands from the repository root. Source ROS 2 in the action-server, runtime, and OpenAPI-export terminals, and ensure `test_msgs/action/Fibonacci` is installed. The HTTP client itself does not need a ROS environment.

Start the existing Fibonacci server in one terminal:

```bash
node example/actions/action_server/action-server-example.mjs
```

Expose that action through the web runtime in another terminal:

```bash
node bin/rclnodejs-web.js --host 127.0.0.1 --port 9000 --http-port 9001 \
  --action /fibonacci=test_msgs/action/Fibonacci
```

The runtime exposes both transports; the URL supplied to the client selects the action transport. The `--http-sse` flag is not needed for actions: it enables HTTP topic subscriptions only. Exposing an action does not start its ROS server. If a request reports `action_unavailable`, check the server and ROS discovery before submitting another goal.

Run the HTTP-only client in a third terminal:

```bash
node example/actions/action_client/http-action-client-example.mjs
```

It sends `{ "order": 5 }`, logs incremental feedback, and finishes with status `succeeded` and result `[0, 1, 1, 2, 3, 5]`. To use a different runtime address:

```bash
node example/actions/action_client/http-action-client-example.mjs http://127.0.0.1:9101
```

From an installed package, import the SDK with `import { connect } from 'rclnodejs/web'`; this checkout example imports its source entry point directly.

### Read the raw SSE stream

The same endpoint can be called without the SDK. `-N` disables curl's output buffering; `--fail-with-body` reports HTTP errors while retaining their JSON body.

```bash
curl --fail-with-body -sS -N http://127.0.0.1:9001/capability/action/fibonacci \
  -H 'content-type: application/json' \
  -d '{"order":3}'
```

The response contains named SSE events with JSON `data:` payloads. A representative response is shown below. ROS delivers feedback and goal replies independently, so `feedback` can arrive before `accepted`; consume events by name rather than requiring this order.

```text
event: accepted
data: {"capability":"/fibonacci"}

event: feedback
data: {"sequence":[0,1,1]}

event: feedback
data: {"sequence":[0,1,1,2]}

event: result
data: {"status":"succeeded","payload":{"sequence":[0,1,1,2]}}

```

Before streaming starts, failures use a non-2xx HTTP status and a JSON body with `ok`, `error`, and `code`. After streaming starts, failures use a terminal SSE `error` event and the HTTP status remains `200`; curl's exit status alone does not indicate that the ROS goal succeeded. The SDK rejects `ros.action()` for request failures or `goal.result` for stream errors. A normal result can have status `canceled` or `aborted`, so also inspect `goal.status`.

### Cancellation and browser use

HTTP action handles do not support `cancel()`; it rejects with `unsupported_kind`. Closing the client or interrupting curl closes the stream but does not cancel the ROS goal. Submit the goal over WebSocket when cancellation is required, using a `ws://` URL or an explicit `{ http, ws }` endpoint pair. See the [SDK action examples](../../web/README.md#actions).

Browser HTTP actions use `fetch()` and `ReadableStream`, which the SDK handles. Native `EventSource` cannot submit the required POST. For a page on another origin, configure the runtime's `--http-cors` option for that origin.

### Export the action contract

```bash
node bin/rclnodejs-web.js openapi --http-port 9001 \
  --action /fibonacci=test_msgs/action/Fibonacci > openapi.json
```

This describes the goal, feedback, result/status, and error payloads without running a ROS node. ROS 2 must still be sourced to load interface metadata. OpenAPI describes the individual SSE event data; consumers need SSE-aware parsing, and API explorers may buffer the response until the goal finishes. Use the SDK or curl to observe live feedback.

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
