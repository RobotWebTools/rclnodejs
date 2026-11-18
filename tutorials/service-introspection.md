# Service Introspection

Service introspection publishes events when a client sends a request or a server handles it. Use these events to inspect calls without adding logging to every request handler.

This feature requires ROS 2 Iron or newer; it is unavailable on Humble. Use the project and service basics from [Writing Your First rclnodejs Nodes](writing-your-first-rclnodejs-nodes.md), with ROS sourced in each terminal.

## Enable Events on the Server

Create `introspection-server.mjs`:

```javascript
import rclnodejs from 'rclnodejs';

if (
  rclnodejs.DistroUtils.getDistroId() <=
  rclnodejs.DistroUtils.getDistroId('humble')
) {
  throw new Error('Service introspection requires ROS 2 Iron or newer');
}

await rclnodejs.init();
const node = rclnodejs.createNode('introspection_server');
const service = node.createService(
  'example_interfaces/srv/AddTwoInts',
  'add_two_ints',
  (request, response) => response.send({ sum: request.a + request.b })
);

service.configureIntrospection(
  node.getClock(),
  rclnodejs.QoS.profileSystemDefault,
  rclnodejs.ServiceIntrospectionStates.CONTENTS
);

console.log('Service introspection enabled on /add_two_ints');
rclnodejs.spin(node);
```

`configureIntrospection()` takes a clock, the event publisher's QoS profile, and an introspection state:

| State      | Events published                           |
| ---------- | ------------------------------------------ |
| `OFF`      | None; this is the default.                 |
| `METADATA` | Event type, timestamp, and client details. |
| `CONTENTS` | Metadata plus request or response data.    |

Introspection must be configured separately on each endpoint. Enabling it on the server does not enable it on clients.

## Watch a Service Call

Start the server:

```bash
node introspection-server.mjs
```

In a second terminal, start monitoring before making a call:

```bash
ros2 topic list --include-hidden-topics
ros2 topic echo /add_two_ints/_service_event example_interfaces/srv/AddTwoInts_Event
```

The event topic is hidden because its final component starts with `_`. Its message type is specific to the service: `AddTwoInts_Event` contains an `info` field of type `service_msgs/msg/ServiceEventInfo`, plus `request` and `response` arrays.

From a third terminal:

```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts '{a: 5, b: 3}'
```

The call returns a sum of 8. The monitor can see the server's request-received and response-sent events, including the corresponding values.

## Enable Client Events

To observe both ends, create `introspection-client.mjs`:

```javascript
import rclnodejs from 'rclnodejs';

await rclnodejs.init();
const node = rclnodejs.createNode('introspection_client');
const client = node.createClient(
  'example_interfaces/srv/AddTwoInts',
  'add_two_ints'
);

client.configureIntrospection(
  node.getClock(),
  rclnodejs.QoS.profileSystemDefault,
  rclnodejs.ServiceIntrospectionStates.CONTENTS
);
rclnodejs.spin(node);

try {
  if (!(await client.waitForService(5000))) {
    throw new Error('Service /add_two_ints is unavailable');
  }
  const response = await client.sendRequestAsync(
    { a: 5n, b: 3n },
    { timeout: 5000 }
  );
  console.log(`Sum: ${response.sum}`);
} finally {
  rclnodejs.shutdown();
}
```

Leave the event monitor running and run:

```bash
node introspection-client.mjs
```

The client prints `Sum: 8` and exits. Both endpoints now publish to the same event topic.

## Read the Events

| `info.event_type` | Meaning             | Published by |
| ----------------- | ------------------- | ------------ |
| 0                 | `REQUEST_SENT`      | Client       |
| 1                 | `REQUEST_RECEIVED`  | Server       |
| 2                 | `RESPONSE_SENT`     | Server       |
| 3                 | `RESPONSE_RECEIVED` | Client       |

In `CONTENTS` mode, request events carry a request and response events carry a response. In `METADATA` mode, both payload arrays are empty.

Use `info.client_gid` together with `info.sequence_number` to correlate a call. A sequence number alone is not unique across clients. Events come from separate publishers, so do not assume a fixed arrival order or exactly four messages for every call. Discovery, QoS, shutdown timing, and when monitoring starts can affect what you receive.

Timestamps come from the clock supplied to each endpoint. Compare timestamps across machines only when those clocks are suitably synchronized.

## Limit What You Publish

Use `METADATA` when payloads are unnecessary. You can call `configureIntrospection()` again on a running endpoint to change its state, or select `OFF` to stop publishing events. These settings affect future events; they do not erase data already received or recorded.

`CONTENTS` adds serialization and transport work and exposes request and response values on a topic. Avoid enabling it indiscriminately for sensitive or high-rate services.

If the monitor is quiet, check the endpoint's state, the full service name, and event-topic QoS compatibility. Service-request QoS and event-publisher QoS are separate settings.

Press **Ctrl+C** to stop the server and monitor when finished.

## Further Reading

- [ROS 2 service introspection](https://docs.ros.org/en/lyrical/Developer-Tools/Introspection-and-analysis/Service-Introspection.html) describes the event protocol.
- [Service-introspection tests](../test/test-service-introspection.js) cover the endpoint states and payloads.
