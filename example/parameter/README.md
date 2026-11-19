# Parameter Examples

Run these commands from the repository root with ROS sourced and `npm install` completed. See the [build requirements](../../docs/BUILDING.md#requirements) for checkout setup.

## Declare and Override Values

```bash
node example/parameter/parameter-declaration-example.mjs
node example/parameter/parameter-override-example.mjs
```

The first program declares `param1` as `hello world`. The second supplies its own ROS argument list and reads back `hello ros2`. Change the `argv` array in that file to try a different override.

Both programs print their results and exit; they do not spin a parameter service for the CLI to call. For a node you can configure with `ros2 param`, use the [parameter tutorial](../../tutorials/parameter-service.md).

## Try a Parameter Client

This example creates its own target node and client, reads and updates several parameters, and exits:

```bash
node example/parameter/parameter-client-advanced-example.mjs
```

It also reads a descriptor and makes a call with a short timeout. A short timeout may or may not expire, depending on how quickly the service responds.

## Read and Watch TurtleSim Parameters

Install the ROS `turtlesim` package. In a ROS-sourced terminal with a graphical session, start it:

```bash
ros2 run turtlesim turtlesim_node
```

Before running the [basic client](parameter-client-basic-example.mjs), add `rclnodejs.spin(node)` after its `createNode()` call so it can process replies. Then run it from another ROS-sourced terminal at the repository root to read and change the blue background value:

```bash
node example/parameter/parameter-client-basic-example.mjs
```

To watch changes instead:

```bash
node example/parameter/parameter-watcher-example.mjs
```

The watcher starts with `background_r` and `background_g`, adds `background_b`, then removes `background_g`. Change the remaining watched values from another terminal:

```bash
ros2 param set /turtlesim background_r 200
ros2 param set /turtlesim background_b 100
```

The watcher prints the updates. Stop it and TurtleSim with Ctrl+C.

See the [parameter tutorial](../../tutorials/parameter-service.md#choose-a-callback) for local validation callbacks and accepted-change notifications. A remote watcher and a local on-set callback serve different purposes.