# Interface Parser Examples

These programs parse a `.msg`, `.srv`, or `.action` file and print its fields. They inspect installed files, not a running ROS graph.

## Run

Install `std_msgs`, `std_srvs`, and `test_msgs`, source ROS, and run `npm install` at the repository root. See the [build requirements](../../docs/BUILDING.md#requirements) for checkout setup.

Run the examples from the repository root:

```bash
AMENT_PREFIX_PATH="$(ros2 pkg prefix std_msgs)" \
  node example/rosidl/rosidl-parse-msg-example.mjs

AMENT_PREFIX_PATH="$(ros2 pkg prefix std_srvs)" \
  node example/rosidl/rosidl-parse-srv-example.mjs

AMENT_PREFIX_PATH="$(ros2 pkg prefix test_msgs)" \
  node example/rosidl/rosidl-parse-action-example.mjs
```

The scripts expect a single prefix in `AMENT_PREFIX_PATH`. These per-command assignments select the right package even when the sourced environment contains several overlay prefixes. They do not change the rest of your shell's environment.

| Program | Interface |
| --- | --- |
| [Message parser](rosidl-parse-msg-example.mjs) | `std_msgs/msg/ColorRGBA` |
| [Service parser](rosidl-parse-srv-example.mjs) | `std_srvs/srv/SetBool` |
| [Action parser](rosidl-parse-action-example.mjs) | `test_msgs/action/Fibonacci` |

## Read the Fields

Each field has a `name`, a `type` object, and `default_value`. The base type is in `field.type.type`; array metadata such as `isArray`, `arraySize`, and `isUpperBound` is also inside `field.type`.

Services return separate request and response definitions. Actions return goal, result, and feedback definitions. For `test_msgs/action/Fibonacci`, the goal field is `order`, and both result and feedback use `sequence`.

See the [parser implementation](../../rosidl_parser/rosidl_parser.cjs) for the returned structures. For generated JavaScript classes and default values instead, use [MessageIntrospector](../message-introspector/README.md).