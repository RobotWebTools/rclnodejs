# Lifecycle Example

The [countdown node](lifecycle-node-example.mjs) configures and activates itself, publishes values from 5 to 0 on `/test`, then deactivates, shuts down, and exits when its own subscriber receives 0.

## Run

With ROS sourced and `npm install` completed, run from the repository root:

```bash
node example/lifecycle/lifecycle-node-example.mjs
```

You need `std_msgs` and `lifecycle_msgs`. See the [build requirements](../../docs/BUILDING.md#requirements) if you need to prepare the checkout.

The output includes `CONFIGURE`, `ACTIVATE`, the countdown messages, `DEACTIVATE`, and `SHUTDOWN`. To watch `/test` independently, start this command in another ROS-sourced terminal before launching the example:

```bash
ros2 topic echo /test std_msgs/msg/String
```

## State and Cleanup

The configure callback creates a lifecycle publisher and a regular subscription. The activate callback explicitly enables the publisher and starts a one-second timer; deactivation disables publishing and cancels that timer.

A lifecycle publisher does not automatically follow the node's state. The registered callbacks handle activation and deactivation. Use `node.currentState` to inspect the state in code.

This program controls its own transitions. For a node that waits for `ros2 lifecycle` commands, or for cleanup and reconfiguration without exiting, use the [lifecycle tutorial](../../tutorials/lifecycle-nodes.md).