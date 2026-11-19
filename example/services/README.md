# Service Examples

Run these commands from the repository root after installing the [build prerequisites](../../docs/BUILDING.md#requirements), sourcing ROS, and running `npm install`. These source examples do not need `npm run build:dist`.

## Add Two Integers

Start the server:

```bash
node example/services/service/service-example.mjs
```

In another ROS-sourced terminal at the repository root, run the callback-based client:

```bash
node example/services/client/client-example.mjs
```

The client sends two random integers, prints their sum, and exits. To try the same call with `async`/`await` and a five-second response timeout:

```bash
node example/services/client/async-client-example.mjs
```

Both clients wait up to one second for service discovery. Start the server first; if discovery times out, run the client again. The server stays running until you press Ctrl+C.

`example_interfaces/srv/AddTwoInts` uses signed 64-bit integers. Use `bigint` values such as `42n` in JavaScript. The callback and Promise APIs are both asynchronous.

You can also call the server from the CLI:

```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts '{a: 42, b: 17}'
```

The response contains `sum=59`.

## Get a Map

With `nav_msgs` installed, start the map server:

```bash
node example/services/service/getmap-service-example.mjs
```

In another ROS-sourced terminal at the repository root:

```bash
node example/services/client/getmap-client-example.mjs
```

The server returns a sample 10-by-10 occupancy grid on `/get_map`. The client prints its dimensions, cell counts, and a text rendering, then exits. Stop the server with Ctrl+C.

## More Examples

- [Request validation](client/client-validation-example.mjs) shows schema checks and rejected requests.
- [Service tutorial](../../tutorials/writing-your-first-rclnodejs-nodes.md#services-requestresponse) explains how to write a client and server.
- [Service introspection](../../tutorials/service-introspection.md) covers the event topics enabled by these client/server examples on Iron and newer.