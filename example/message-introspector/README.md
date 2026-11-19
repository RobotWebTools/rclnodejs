# Message Introspection

`MessageIntrospector` reads generated message definitions. The [example](message-introspector-example.mjs) prints fields and defaults for `Twist`, `String`, and `JointState`.

## Run

Install `geometry_msgs`, `std_msgs`, and `sensor_msgs`, source ROS, and run `npm install` at the repository root. Then run:

```bash
node example/message-introspector/message-introspector-example.mjs
```

The output includes `Twist fields: [ 'linear', 'angular' ]` and the default vectors. The program exits after printing the definitions; it does not need a publisher.

## Available Properties

Create an instance with `new rclnodejs.MessageIntrospector('geometry_msgs/msg/Twist')` in an application that imports rclnodejs.

| Property | Value |
| --- | --- |
| `typeName` | The supplied type name. |
| `fields` | The field names. |
| `defaults` | A deep copy of the message's default values. |
| `schema` | The generated `ROSMessageDef` metadata. |
| `typeClass` | The generated message constructor. |

The defaults are cached internally, but each access returns a fresh copy. Custom interfaces must be installed and generated before they can be loaded.

To ask another running node about its advertised interface instead, see the [type description service](../../tutorials/type-description-service.md).