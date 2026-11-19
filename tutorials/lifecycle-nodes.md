# Lifecycle Nodes

A lifecycle node separates configuration from operation. For example, a camera can allocate buffers when configured, start capture when activated, and stop capture without losing its configuration when deactivated.

This guide builds on [Writing Your First rclnodejs Nodes](writing-your-first-rclnodejs-nodes.md). Use the same npm project and source ROS in each terminal. You will also need the `lifecycle_msgs` package.

## States and Transitions

| State        | Meaning in the example below                         |
| ------------ | ---------------------------------------------------- |
| Unconfigured | The node exists but has no application resources.    |
| Inactive     | The publisher and timer exist, but no messages flow. |
| Active       | The timer publishes a status message each second.    |
| Finalized    | The lifecycle has ended and resources are released.  |

The normal path is `configure` followed by `activate`. Use `deactivate` to return to inactive, then `cleanup` to return to unconfigured. You can `shutdown` from unconfigured, inactive, or active to reach finalized.

These states describe your application's behavior. They do not automatically stop ordinary subscriptions or timers; your transition callbacks must do that work.

## Write the Node

Create `lifecycle-node.mjs`:

```javascript
import rclnodejs from 'rclnodejs';

await rclnodejs.init();
const node = rclnodejs.createLifecycleNode('managed_talker');
const { SUCCESS } = rclnodejs.lifecycle.CallbackReturnCode;
let publisher;
let timer;
let counter = 0;

function releaseResources() {
  if (timer) {
    node.destroyTimer(timer);
    timer = null;
  }
  if (publisher) {
    node.destroyPublisher(publisher);
    publisher = null;
  }
}

node.registerOnConfigure(() => {
  publisher = node.createLifecyclePublisher('std_msgs/msg/String', 'status');
  timer = node.createTimer(
    1_000_000_000n,
    () => publisher.publish({ data: `Status ${counter++}` }),
    { autostart: false }
  );
  counter = 0;
  console.log('Configured');
  return SUCCESS;
});

node.registerOnActivate(() => {
  publisher.activate();
  timer.reset();
  console.log('Active');
  return SUCCESS;
});

node.registerOnDeactivate(() => {
  timer.cancel();
  publisher.deactivate();
  console.log('Inactive');
  return SUCCESS;
});

node.registerOnCleanup(() => {
  releaseResources();
  console.log('Unconfigured');
  return SUCCESS;
});

node.registerOnShutdown(() => {
  releaseResources();
  console.log('Finalized');
  return SUCCESS;
});

node.registerOnError(() => {
  releaseResources();
  console.error('Transition failed; application resources released');
  return SUCCESS;
});

console.log(`State: ${node.currentState.label}`);
rclnodejs.spin(node);
```

The lifecycle publisher starts disabled. This example explicitly activates and deactivates it in the matching callbacks. A disabled lifecycle publisher ignores `publish()` calls; it does not stop the rest of the node.

Creating the timer during configuration and resetting it during activation lets the node reuse it across deactivate/activate cycles. Cleanup destroys both entities, so the node can be configured again without accumulating timers or publishers.

## Control It from the CLI

Start the program:

```bash
node lifecycle-node.mjs
```

In another terminal, inspect and activate it:

```bash
ros2 lifecycle get /managed_talker
ros2 lifecycle list /managed_talker
ros2 lifecycle set /managed_talker configure
ros2 lifecycle set /managed_talker activate
ros2 topic echo /status std_msgs/msg/String --once
```

The initial state is `unconfigured`. After activation, the topic receives `Status 0`, `Status 1`, and so on. The first message you see depends on when the subscriber joins.

Stop publishing, release the resources, then configure the node again:

```bash
ros2 lifecycle set /managed_talker deactivate
ros2 lifecycle set /managed_talker cleanup
ros2 lifecycle set /managed_talker configure
ros2 lifecycle set /managed_talker activate
```

You cannot activate an unconfigured node directly. Use `ros2 lifecycle list` to see which transitions are valid from its current state.

To finish the lifecycle:

```bash
ros2 lifecycle set /managed_talker shutdown
ros2 lifecycle get /managed_talker
```

The state becomes `finalized`. The process remains running so you can inspect it; press **Ctrl+C** to stop it. A lifecycle transition is distinct from `rclnodejs.shutdown()`, which destroys the ROS context and its nodes. SIGINT cleanup does not replace your transition callbacks for resources outside ROS, such as open files or hardware connections.

## Handle Transition Results

Callbacks must synchronously return `SUCCESS`, `FAILURE`, or `ERROR` from `rclnodejs.lifecycle.CallbackReturnCode`; do not register an `async` function as a transition callback.

Use `FAILURE` when the requested transition cannot complete, such as an unavailable device during configuration. `ERROR` enters error processing and invokes the registered error callback. Returning `SUCCESS` from that error callback returns the node to unconfigured; cleanup must have left it ready to configure again.

You can also call `node.configure()`, `node.activate()`, and the other transitions locally. They return the resulting state; read `node.currentState` to inspect it. For remote control from another node, use the `lifecycle_msgs/srv/ChangeState` service at `/managed_talker/change_state` with the asynchronous client pattern from the [first-nodes guide](writing-your-first-rclnodejs-nodes.md#write-the-client).
