# TypeScript Actions Demo

The [client](src/client.ts) asks the [server](src/server.ts) for Fibonacci(10), prints feedback and the result, then exits. The server handles cancellation requests.

## Run

After [TypeScript setup](../README.md#setup) with `test_msgs` installed, run from the repository root:

```bash
cd demo/typescript/actions
npm install --ignore-scripts
npm run start:both
```

`start:both` builds and runs both nodes. To run them separately, start `npm run start:server`, then `npm run start:client` in another terminal in the demo directory. Stop the server with Ctrl+C.

## Send a Goal

Leave the server running and send a goal from another terminal:

```bash
ros2 action send_goal /fibonacci test_msgs/action/Fibonacci '{order: 5}' --feedback
```

The server sends feedback as it works and returns `[0, 1, 1, 2, 3, 5]`. Both programs use the constructor from `rclnodejs.require()` to infer goal, feedback, and result types.

For a walkthrough of goals and cancellation, see the [actions tutorial](../../../tutorials/actionlib.md).