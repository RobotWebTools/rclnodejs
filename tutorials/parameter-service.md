# Parameters

Parameters hold a node's configuration. You declare them in code, then read or change them locally or through ROS 2 services. Each parameter belongs to one node and has a name, type, and value.

Use the project from [Writing Your First rclnodejs Nodes](writing-your-first-rclnodejs-nodes.md), with ROS sourced in each terminal. The examples below use ES modules.

## Declare and Validate Parameters

Create `parameters.mjs`:

```javascript
import rclnodejs from 'rclnodejs';

const { Parameter, ParameterType, ParameterDescriptor, FloatingPointRange } =
  rclnodejs;

await rclnodejs.init();
const node = rclnodejs.createNode('configurable_node');

node.declareParameter(
  new Parameter('robot_name', ParameterType.PARAMETER_STRING, 'rover')
);

const speedDescriptor = new ParameterDescriptor(
  'max_speed',
  ParameterType.PARAMETER_DOUBLE,
  'Maximum speed in m/s'
);
speedDescriptor.range = new FloatingPointRange(0.0, 5.0, 0.0);
node.declareParameter(
  new Parameter('max_speed', ParameterType.PARAMETER_DOUBLE, 1.0),
  speedDescriptor
);

node.addOnSetParametersCallback((parameters) => {
  for (const parameter of parameters) {
    if (
      parameter.name === 'robot_name' &&
      (typeof parameter.value !== 'string' || parameter.value.trim() === '')
    ) {
      return { successful: false, reason: 'robot_name must not be empty' };
    }
  }
  return { successful: true, reason: '' };
});

node.addPostSetParametersCallback((parameters) => {
  for (const parameter of parameters) {
    console.log(`${parameter.name} changed to ${parameter.value}`);
  }
});

console.log(`Robot: ${node.getParameter('robot_name').value}`);
console.log(`Maximum speed: ${node.getParameter('max_speed').value}`);
rclnodejs.spin(node);
```

The descriptor accepts speeds from 0 to 5; a step of 0 allows any value within that range. The on-set callback rejects blank robot names and requests to unset the name.

Parameters must be declared before they can be updated. Their types remain fixed by the declarations. ROS 2 supports booleans, integers, doubles, strings, and array types; use `BigInt` values such as `10n` for integer parameters.

## Use the ROS 2 CLI

Start the node:

```bash
node parameters.mjs
```

In another terminal:

```bash
ros2 param list /configurable_node
ros2 param get /configurable_node max_speed
ros2 param describe /configurable_node max_speed
ros2 param set /configurable_node max_speed 2.5
ros2 param set /configurable_node robot_name scout
```

The node prints accepted changes. Try setting `max_speed` to `9.0`: the update is rejected, and its previous value remains in place. Use a decimal value for this parameter; `2` is an integer in ROS command-line YAML, while `2.0` is a double.

Parameter services are enabled by default. The node must spin to handle requests from `ros2 param` or another node. Local calls such as `getParameter()` and `setParameter()` do not require spinning.

Press **Ctrl+C** to stop the node.

## Override Startup Values

Restart the same program with ROS arguments:

```bash
node parameters.mjs --ros-args -p robot_name:=scout -p max_speed:=2.5
```

`init()` reads the process arguments by default. `declareParameter()` uses matching overrides instead of the defaults in the code, while still checking the descriptor. In this example, the custom on-set callback is registered after declaration, so it only validates later updates.

You can also save and reload a running node's parameters:

```bash
ros2 param dump /configurable_node > parameters.yaml
ros2 param load /configurable_node parameters.yaml
```

Loading into a running node applies each update separately. A dump can include read-only parameters, which the node will reject on reload. It does not replace the declarations in the program.

## Update Parameters in Code

To update related settings together, add this before `spin()` in the example:

```javascript
const result = node.setParametersAtomically([
  new Parameter('robot_name', ParameterType.PARAMETER_STRING, 'scout'),
  new Parameter('max_speed', ParameterType.PARAMETER_DOUBLE, 2.5),
]);

if (!result.successful) {
  console.error(result.reason);
}
```

The whole update is rejected if either value fails validation. By contrast, `setParameters()` applies each parameter separately and returns a result for each one. `setParameter()` updates a single value.

## Choose a Callback

- `addPreSetParametersCallback()` can change the proposed list before validation. Return an array of parameters; an empty array rejects the update.
- `addOnSetParametersCallback()` accepts or rejects a proposed change. Return `{ successful, reason }` and avoid side effects: another validator may still reject it.
- `addPostSetParametersCallback()` runs after a successful change. Use it to update application state or log accepted values; its return value is ignored.

For changes on other nodes, use parameter events or a parameter client rather than an on-set callback on the local node. You can inspect the event topic with:

```bash
ros2 topic echo /parameter_events
```
