# Actions

Use an action when a task takes time and the caller needs progress updates or cancellation. The client sends a goal, receives feedback while it runs, and asks for the final result. A server may reject a goal or a cancellation request.

Start with the project from [Writing Your First rclnodejs Nodes](writing-your-first-rclnodejs-nodes.md). This example also needs `test_msgs/action/Fibonacci`; install the ROS `test_msgs` package before installing rclnodejs, or rerun `npx generate-ros-messages` after adding it.

## Write the Server

Create `action-server.mjs`:

```javascript
import { setTimeout as delay } from 'node:timers/promises';
import rclnodejs from 'rclnodejs';

await rclnodejs.init();
const node = rclnodejs.createNode('fibonacci_server');
const Fibonacci = rclnodejs.require('test_msgs/action/Fibonacci');

async function execute(goalHandle) {
  const sequence = [0, 1];
  const result = new Fibonacci.Result();

  for (let index = 1; index < goalHandle.request.order; index++) {
    await delay(500);
    if (goalHandle.isCancelRequested) {
      goalHandle.canceled();
      result.sequence = sequence;
      return result;
    }

    sequence.push(sequence.at(-1) + sequence.at(-2));
    const feedback = new Fibonacci.Feedback();
    feedback.sequence = sequence;
    goalHandle.publishFeedback(feedback);
  }

  goalHandle.succeed();
  result.sequence = sequence;
  return result;
}

new rclnodejs.ActionServer(
  node,
  'test_msgs/action/Fibonacci',
  'fibonacci',
  execute,
  (goal) =>
    goal.order >= 2 && goal.order <= 20
      ? rclnodejs.GoalResponse.ACCEPT
      : rclnodejs.GoalResponse.REJECT,
  undefined,
  () => rclnodejs.CancelResponse.ACCEPT
);

console.log('Action /fibonacci is ready');
rclnodejs.spin(node);
```

The constructor arguments after `execute` are the goal callback, accepted-goal callback, and cancel callback. Passing `undefined` keeps the default accepted-goal behavior: start execution immediately.

This server accepts orders from 2 to 20 and returns Fibonacci numbers from index 0 through the requested order. Each half-second delay stands in for a unit of work. It also lets JavaScript process new requests and cancellation; an `async` function that runs a long CPU loop without yielding would still block the event loop.

## Write the Client

Create `action-client.mjs`:

```javascript
import { setTimeout as delay } from 'node:timers/promises';
import rclnodejs from 'rclnodejs';

await rclnodejs.init();
const node = rclnodejs.createNode('fibonacci_client');
const client = new rclnodejs.ActionClient(
  node,
  'test_msgs/action/Fibonacci',
  'fibonacci'
);
rclnodejs.spin(node);

try {
  if (!(await client.waitForServer(5000))) {
    throw new Error('Action server is unavailable');
  }

  const goal = await client.sendGoal({ order: 5 }, (feedback) => {
    console.log(`Feedback: ${Array.from(feedback.sequence).join(', ')}`);
  });
  if (!goal.isAccepted()) {
    throw new Error('Goal was rejected');
  }

  if (process.argv.includes('--cancel')) {
    await delay(750);
    const response = await goal.cancelGoal();
    console.log(
      `Cancellation accepted: ${response.goals_canceling.length > 0}`
    );
  }

  const result = await goal.getResult();
  const sequence = Array.from(result.sequence).join(', ');
  if (goal.isSucceeded()) {
    console.log(`Result: ${sequence}`);
  } else if (goal.isCanceled()) {
    console.log(`Canceled with partial result: ${sequence}`);
  } else {
    throw new Error(`Goal ended with status ${goal.status}`);
  }
} finally {
  rclnodejs.shutdown();
}
```

`waitForServer(5000)` returns `false` if discovery times out. That timeout does not limit goal execution or the wait for a result. Spinning is needed while awaiting goal responses, feedback, and results. The `finally` block also runs if the server is unavailable or rejects the goal.

## Run Both Nodes

Source ROS in both terminals and start the server:

```bash
node action-server.mjs
```

In the other terminal:

```bash
node action-client.mjs
```

The client prints feedback and finishes with:

```text
Result: 0, 1, 1, 2, 3, 5
```

You can inspect or call the action without the JavaScript client:

```bash
ros2 action list -t
ros2 action info /fibonacci
ros2 action send_goal /fibonacci test_msgs/action/Fibonacci '{order: 5}' --feedback
```

## Try Cancellation

With the server still running:

```bash
node action-client.mjs --cancel
```

The client requests cancellation after 750 ms. Accepting that request does not stop the task by itself: the execute callback checks `isCancelRequested`, stops its work, calls `canceled()`, and returns a partial result. The client still waits for `getResult()` to learn the terminal status.

For real work, check for cancellation at points where it is safe to stop. If execution fails instead, call `abort()` and return a result that explains the failure through your action's fields. Call `succeed()` only after the task has completed.

Press **Ctrl+C** to stop the server when finished.
