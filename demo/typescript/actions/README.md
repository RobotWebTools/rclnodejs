# TypeScript Actions Demo for rclnodejs

This demo showcases how to use ROS2 actions with rclnodejs in TypeScript. It includes both an action client and server implementation using the Fibonacci action as an example.

## What are ROS2 Actions?

Actions in ROS2 are a communication pattern for long-running tasks that:

- Have a **goal** (what you want to achieve)
- Provide **feedback** during execution (progress updates)
- Return a **result** when completed (final outcome)
- Can be **canceled** while running

Actions are perfect for tasks like navigation, manipulation, or any long-running computation where you need progress updates.

## Demo Overview

This demo implements a Fibonacci sequence calculator using ROS2 actions:

- **Action Server** (`server.ts`): Calculates Fibonacci sequences up to a given order
- **Action Client** (`client.ts`): Sends goals to the server and receives feedback

### Features

- ✅ **Goal handling**: Accept/reject goals based on input validation
- ✅ **Feedback**: Real-time progress updates during calculation
- ✅ **Result**: Final Fibonacci sequence
- ✅ **Cancellation**: Support for canceling running goals
- ✅ **Error handling**: Graceful error handling and logging
- ✅ **TypeScript**: Full type safety and modern TypeScript features

## Project Structure

```
demo/typescript/actions/
├── src/
│   ├── client.ts          # Action client implementation
│   └── server.ts          # Action server implementation
├── package.json           # Project configuration
├── tsconfig.json          # TypeScript configuration
├── .gitignore            # Git ignore rules
└── README.md             # This file
```

## Prerequisites

Before running this demo, ensure you have:

1. **ROS2** installed (tested with ROS2 Humble/Jazzy/Kilted/Lyrical)
2. **Node.js** (version 20.20.2 or higher)
3. **rclnodejs** built and configured in the parent directory (`../../`)
4. **test_msgs** package available (usually included with ROS2)

**Important**: This demo uses rclnodejs as a peer dependency, so you must ensure that the main rclnodejs package in the parent directory is properly built and configured with ROS2 before running this demo.

## Setup and Installation

1. **Ensure ROS2 is sourced**:

   ```bash
   source /opt/ros/lyrical/setup.bash  # or your ROS2 distribution
   ```

2. **Build the main rclnodejs package** (if not already done):

   ```bash
   cd ../../  # Go to main rclnodejs directory
   npm install
   npm run build
   ```

3. **Navigate to the demo directory**:

   ```bash
   cd demo/typescript/actions
   ```

4. **Install dependencies**:

   This demo depends on the **local rclnodejs** in this repository
   (`"rclnodejs": "file:../../.."`). Install with `--ignore-scripts` so npm
   links the local package without trying to rebuild its native addon (the
   prebuilt binary in the repo is reused):

   ```bash
   npm install --ignore-scripts
   ```

5. **Build the TypeScript code**:
   ```bash
   npm run build
   ```

## Running the Demo

### Option 1: Run Server and Client Separately

1. **Start the action server** (in terminal 1):

   ```bash
   npm run start:server
   ```

   You should see output like:

   ```
   🚀 Starting TypeScript Action Server Demo...
   ✓ rclnodejs initialized
   ✓ Created node: /ts_action_server_demo
   ✓ Fibonacci action server created on topic 'fibonacci'
   ✓ Fibonacci action server is ready to receive goals
   ```

2. **Start the action client** (in terminal 2):

   ```bash
   npm run start:client
   ```

   You should see the client sending a goal and receiving feedback:

   ```
   🚀 Starting TypeScript Action Client Demo...
   ✓ rclnodejs initialized
   ✓ Created node: /ts_action_client_demo
   ✓ Action server is available
   Sending goal request for Fibonacci(10)...
   ✓ Goal accepted by server
   📊 Received feedback: [0, 1]
   📊 Received feedback: [0, 1, 1]
   ...
   ✓ Goal succeeded! Fibonacci(10) = 0,1,1,2,3,5,8,13,21,34,55
   ```

### Option 2: Run Both Simultaneously

Run both server and client together using concurrently:

```bash
npm run start:both
```

This will start both the server and client in parallel, showing interleaved output.

### Option 3: Development Mode (with ts-node)

For development with hot reloading:

1. **Server**:

   ```bash
   npm run dev:server
   ```

2. **Client**:
   ```bash
   npm run dev:client
   ```

## Testing with ROS2 CLI Tools

You can also test the action server using ROS2 command-line tools:

1. **Start the server**:

   ```bash
   npm run start:server
   ```

2. **Send a goal using ros2 action**:

   ```bash
   ros2 action send_goal /fibonacci test_msgs/action/Fibonacci "{order: 15}"
   ```

3. **List available actions**:

   ```bash
   ros2 action list
   ```

4. **Get action info**:
   ```bash
   ros2 action info /fibonacci
   ```

## Available Scripts

- `npm run build` - Compile TypeScript to JavaScript
- `npm run clean` - Remove compiled files
- `npm run start:server` - Run the action server
- `npm run start:client` - Run the action client
- `npm run start:both` - Run both server and client concurrently
- `npm run dev:server` - Run server in development mode
- `npm run dev:client` - Run client in development mode
- `npm run check-types` - Type check without compilation

## Understanding the Code

Both the server and client show the **two equivalent, fully type-safe ways**
to identify an action type:

- **String name**: pass the type string, e.g. `'test_msgs/action/Fibonacci'`.
- **Action class**: obtain the constructor with `rclnodejs.require(...)` and
  pass it directly. TypeScript then infers the goal, feedback and result types
  from the constructor, so no explicit `any` annotations are needed.

This demo uses the **action class** form. The constructor is loaded once at
module scope and reused for the type annotations via `typeof Fibonacci`:

```typescript
const Fibonacci = rclnodejs.require('test_msgs/action/Fibonacci');
```

### Action Server (`server.ts`)

The action server implements three main callbacks, all typed from
`typeof Fibonacci`:

1. **Goal Callback**: Decides whether to accept or reject incoming goals

   ```typescript
   goalCallback(
     goal: rclnodejs.ActionGoal<typeof Fibonacci>
   ): rclnodejs.GoalResponse {
     // Validate the goal (goal.order is typed) and return ACCEPT or REJECT
   }
   ```

2. **Execute Callback**: Performs the actual work (Fibonacci calculation)

   ```typescript
   async executeCallback(
     goalHandle: rclnodejs.ServerGoalHandle<typeof Fibonacci>
   ): Promise<rclnodejs.ActionResult<typeof Fibonacci>> {
     // Calculate the sequence and publish typed feedback
   }
   ```

3. **Cancel Callback**: Handles goal cancellation requests

   ```typescript
   cancelCallback(
     goalHandle: rclnodejs.ServerGoalHandle<typeof Fibonacci> | undefined
   ): rclnodejs.CancelResponse {
     // Return ACCEPT to allow cancellation
   }
   ```

### Action Client (`client.ts`)

The action client:

1. Waits for the action server to be available
2. Creates and sends a goal (`new Fibonacci.Goal()`)
3. Handles typed feedback during execution
4. Processes the final result

```typescript
const goal = new Fibonacci.Goal();
goal.order = FIBONACCI_ORDER;

const goalHandle = await this.actionClient.sendGoal(goal, (feedback) =>
  this.feedbackCallback(feedback)
);

private feedbackCallback(
  feedback: rclnodejs.ActionFeedback<typeof Fibonacci>
): void {
  // feedback.sequence is typed
}
```

## Customization

You can modify the demo to:

- **Change the Fibonacci order**: Edit `FIBONACCI_ORDER` in `client.ts`
- **Adjust timing**: Modify the delay in the server's execute callback
- **Add validation**: Enhance goal validation in the server
- **Handle errors**: Add more sophisticated error handling

## Troubleshooting

### Common Issues

1. **"Cannot find module 'rclnodejs'"**:
   - Ensure rclnodejs is properly built in the parent directory
   - Run `npm install` in the main rclnodejs directory

2. **"Action server not available"**:
   - Make sure the action server is running before starting the client
   - Check that both nodes are using the same action name

3. **TypeScript compilation errors**:
   - Run `npm run check-types` to see detailed type errors
   - Ensure all dependencies are installed: `npm install`

4. **ROS2 environment not sourced**:
   ```bash
   source /opt/ros/lyrical/setup.bash  # or your ROS2 distribution
   ```

### Debugging

Enable debug logging by setting the environment variable:

```bash
export RCUTILS_LOGGING_SEVERITY=DEBUG
npm run start:server
```

## Next Steps

After exploring this demo, you might want to:

1. **Create custom actions**: Define your own `.action` files
2. **Handle multiple goals**: Implement concurrent goal handling
3. **Add persistence**: Store goal state across restarts
4. **Integrate with other nodes**: Combine actions with topics and services
5. **Add visualization**: Create a web interface to monitor action progress

## Related Examples

- **Topics Demo**: `../topics/` - Publisher/Subscriber pattern
- **Services Demo**: `../services/` - Request/Response pattern
- **JavaScript Actions**: `../../example/actions/` - JavaScript implementation

## Resources

- [ROS2 Actions Documentation](https://docs.ros2.org/latest/Tutorials/Understanding-ROS2-Actions.html)
- [rclnodejs Documentation](https://github.com/RobotWebTools/rclnodejs)
- [test_msgs Package](https://github.com/ros2/rcl_interfaces/tree/master/test_msgs)

---

**Happy coding with ROS2 Actions and TypeScript! 🚀**
