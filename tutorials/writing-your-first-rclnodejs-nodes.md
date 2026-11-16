# Writing Your First rclnodejs Nodes

Create a Node.js project, run your first ROS 2 node, then connect nodes using topics and services. The examples use JavaScript ES modules and the installed `rclnodejs` package.

## Prerequisites

- ROS 2 with `std_msgs` and `example_interfaces` installed.
- Node.js **20.20.2 or newer**, npm, and the [native build prerequisites](../docs/BUILDING.md#requirements).
- Basic familiarity with JavaScript functions, callbacks, and `async`/`await`.

The commands below use Linux and ROS 2 Lyrical, the latest LTS release. Replace `lyrical` with your installed distro, and source ROS in every terminal used for this tutorial.

## Create a Project

```bash
source /opt/ros/lyrical/setup.bash
mkdir rclnodejs-tutorial
cd rclnodejs-tutorial
npm init -y
npm install rclnodejs
```

A plain npm project is enough for these examples; a `colcon` workspace is not required. The `.mjs` extension enables ES modules without changing the project's package configuration.

## Run Your First Node

Create `first-node.mjs`:

```javascript
import rclnodejs from 'rclnodejs';

await rclnodejs.init();
const node = rclnodejs.createNode('first_node');
console.log('first_node is running');
rclnodejs.spin(node);
```

Run it from the project directory:

```bash
node first-node.mjs
```

In another terminal, source ROS and inspect the node:

```bash
source /opt/ros/lyrical/setup.bash
ros2 node list
ros2 node info /first_node
```

`init()` initializes ROS, `createNode()` gives the node a discoverable name, and `spin()` enables ROS callback processing. Native waiting runs in the background; JavaScript callbacks run on the JS event-loop thread. Keep callbacks short so they do not delay other callbacks.

Press **Ctrl+C** to stop the node. By default, rclnodejs shuts down its contexts and nodes on SIGINT.

## Choose a Communication Pattern

| Pattern | Purpose                                          | Example                                      |
| ------- | ------------------------------------------------ | -------------------------------------------- |
| Topic   | Publish messages to any matching subscribers     | Sensor readings or robot status              |
| Service | Send a request and receive a response            | Query a value or perform a short calculation |
| Action  | Run a longer task with feedback and cancellation | Navigate to a goal                           |

A node can use more than one pattern. Actions are covered in the [action tutorial](actionlib.md).

## Topics (Publish/Subscribe)

A publisher sends messages to a named topic; subscriptions receive messages with the matching topic name, message type, and compatible QoS settings. Publishers and subscribers can run in separate processes.

### Write the Publisher

Create `publisher.mjs`:

```javascript
import rclnodejs from 'rclnodejs';

await rclnodejs.init();
const node = rclnodejs.createNode('talker');
const publisher = node.createPublisher('std_msgs/msg/String', 'hello');

let counter = 0;
node.createTimer(1_000_000_000n, () => {
  const data = `Hello ROS ${counter++}`;
  publisher.publish({ data });
  console.log(`Publishing: ${data}`);
});

rclnodejs.spin(node);
```

`std_msgs/msg/String` has one field, `data`. The timer period is in **nanoseconds**: `1_000_000_000n` is one second. The `n` suffix creates a JavaScript `BigInt`.

### Write the Subscriber

Create `subscriber.mjs`:

```javascript
import rclnodejs from 'rclnodejs';

await rclnodejs.init();
const node = rclnodejs.createNode('listener');
node.createSubscription('std_msgs/msg/String', 'hello', (message) => {
  console.log(`Received: ${message.data}`);
});

rclnodejs.spin(node);
```

The callback runs when a message arrives. Both examples use the topic `hello`, which resolves to `/hello` because the nodes have no namespace.

### Run and Inspect

In two terminals, source ROS and enter the project directory. Start the subscriber in one and the publisher in the other:

```bash
node subscriber.mjs
```

```bash
node publisher.mjs
```

After discovery completes, the subscriber should print:

```text
Received: Hello ROS 0
Received: Hello ROS 1
Received: Hello ROS 2
```

The first counter you receive may be higher if the publisher started earlier. Default volatile durability does not replay earlier messages to late subscribers.

From a third terminal with ROS sourced:

```bash
ros2 topic info /hello
ros2 interface show std_msgs/msg/String
ros2 topic echo /hello std_msgs/msg/String
```

These commands inspect the endpoints, show the message fields, and receive messages without a Node.js subscriber. Stop the running examples with **Ctrl+C**.

## Services (Request/Response)

Services have separate request and response types. rclnodejs supports asynchronous calls, so waiting for a response does not need to block the JS event loop. Use services for short operations; use actions for longer tasks that need feedback or cancellation.

### Write the Server

Create `service-server.mjs`:

```javascript
import rclnodejs from 'rclnodejs';

await rclnodejs.init();
const node = rclnodejs.createNode('adder');
node.createService(
  'example_interfaces/srv/AddTwoInts',
  'add_two_ints',
  (request, response) => {
    response.send({ sum: request.a + request.b });
  }
);

console.log('Service /add_two_ints is ready');
rclnodejs.spin(node);
```

### Write the Client

Create `service-client.mjs`:

```javascript
import rclnodejs from 'rclnodejs';

await rclnodejs.init();
const node = rclnodejs.createNode('add_client');
const client = node.createClient(
  'example_interfaces/srv/AddTwoInts',
  'add_two_ints'
);
rclnodejs.spin(node);

try {
  if (!(await client.waitForService(5000))) {
    throw new Error('Service /add_two_ints is unavailable');
  }
  const response = await client.sendRequestAsync(
    { a: 10n, b: 15n },
    { timeout: 5000 }
  );
  console.log(`Sum: ${response.sum}`);
} finally {
  rclnodejs.shutdown();
}
```

`AddTwoInts` uses signed 64-bit integers. `BigInt` inputs preserve integer precision. `waitForService()` waits for discovery, while `sendRequestAsync()` waits for the response; both timeouts above are in **milliseconds**. Spinning lets the response callback run while the client awaits its Promise.

### Run and Inspect

Start the server and client in separate terminals, with ROS sourced and the project directory selected:

```bash
node service-server.mjs
```

```bash
node service-client.mjs
```

The client prints `Sum: 25`, shuts down, and exits. The server continues running until you press **Ctrl+C**.

You can also call the server using the ROS 2 CLI:

```bash
ros2 service type /add_two_ints
ros2 interface show example_interfaces/srv/AddTwoInts
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts '{a: 5, b: 10}'
```

The response should contain `sum=15`.

## Next Steps

- [Parameter service](parameter-service.md): configure a node at runtime.
- [Lifecycle nodes](lifecycle-nodes.md): control startup and shutdown states.
- [Actions](actionlib.md): add feedback and cancellation to longer tasks.
- [More examples](../example/): explore additional messages and QoS configurations.
