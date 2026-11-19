# Action Examples

These examples use `test_msgs/action/Fibonacci` on `/fibonacci`. Install `test_msgs`, source ROS, and run `npm install` at the repository root before starting. See the [build requirements](../../docs/BUILDING.md#requirements) for checkout setup.

## Run a Server and Client

From the repository root:

```bash
node example/actions/action_server/action-server-example.mjs
```

In another ROS-sourced terminal at the repository root:

```bash
node example/actions/action_client/action-client-example.mjs
```

The client sends order 10, prints feedback as the server works, and finishes with `0,1,1,2,3,5,8,13,21,34,55`. It then exits. The server stays running until you press Ctrl+C.

You can also send a goal from the CLI:

```bash
ros2 action send_goal /fibonacci test_msgs/action/Fibonacci '{order: 5}' --feedback
```

## Other Clients and Servers

Run only one of these servers on `/fibonacci` at a time.

| Example | What changes |
| --- | --- |
| [Cancellation client](action_client/action-client-cancel-example.mjs) | Requests cancellation shortly after the goal is accepted. |
| [Deferred server](action_server/action-server-defer-example.mjs) | Waits three seconds before starting an accepted goal. |
| [Single-goal server](action_server/action-server-single-goal-example.mjs) | Aborts the previous goal when accepting a new one. |
| [Goal validation](action_client/action-client-validation-example.mjs) | Demonstrates schema checks using `action_tutorials_interfaces/action/Fibonacci`; install that interface package first. |

The cancellation client reports whether the server accepted the cancel request; that is not the same as waiting for the goal's final canceled result. For a client that waits for the terminal status, see the [actions tutorial](../../tutorials/actionlib.md#try-cancellation).

Use `ros2 action list -t` and `ros2 action info /fibonacci` to inspect the action. Its result is retrieved through a service, not an `/_action/result` topic.