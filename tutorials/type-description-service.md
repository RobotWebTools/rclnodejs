# Type Description Service

`ros2 interface show` reads an interface installed on your machine. The type description service asks a running node for the fields and, optionally, the source of a type it knows. It is useful when you want to inspect the definition advertised by a particular endpoint.

Use ROS 2 Jazzy or newer for this example. Start the publisher from [Writing Your First rclnodejs Nodes](writing-your-first-rclnodejs-nodes.md#write-the-publisher). It runs as `/talker` and publishes `std_msgs/msg/String` on `/hello`.

## Find the Service

With ROS sourced in both terminals, run the publisher:

```bash
node publisher.mjs
```

Then inspect the node and its topic:

```bash
ros2 node info /talker
ros2 topic info /hello --verbose
ros2 service type /talker/get_type_description
```

The service type is `type_description_interfaces/srv/GetTypeDescription`. Its name is relative to the node, so a talker under the namespace `/demo` would offer `/demo/talker/get_type_description`.

## Query the Advertised Type

Create `describe-type.mjs` in the same npm project:

```javascript
import { setTimeout as delay } from 'node:timers/promises';
import rclnodejs from 'rclnodejs';

await rclnodejs.init();
const node = rclnodejs.createNode('type_query');

try {
  const client = node.createClient(
    'type_description_interfaces/srv/GetTypeDescription',
    '/talker/get_type_description'
  );
  rclnodejs.spin(node);

  if (!(await client.waitForService(5000))) {
    throw new Error('Type description service is unavailable');
  }

  const findPublisher = () =>
    node
      .getPublishersInfoByTopic('/hello', false)
      .find(
        (endpoint) =>
          endpoint.node_name === 'talker' && endpoint.node_namespace === '/'
      );

  const deadline = Date.now() + 5000;
  let publisher = findPublisher();
  while (!publisher && Date.now() < deadline) {
    await delay(50);
    publisher = findPublisher();
  }
  const hash = publisher?.topic_type_hash;
  if (!hash || hash.version === 0) {
    throw new Error('No type hash advertised by /talker on /hello');
  }

  const version = hash.version.toString(16).padStart(2, '0');
  const typeHash = `RIHS${version}_${Buffer.from(hash.value).toString('hex')}`;
  const response = await client.sendRequestAsync(
    {
      type_name: publisher.topic_type,
      type_hash: typeHash,
      include_type_sources: true,
    },
    { timeout: 5000 }
  );
  if (!response.successful) {
    throw new Error(response.failure_reason);
  }

  const description = response.type_description.type_description;
  console.log(description.type_name);
  console.log(
    `Fields: ${description.fields.map((field) => field.name).join(', ')}`
  );

  const source = response.type_sources.find(
    (entry) => entry.type_name === description.type_name
  );
  if (source) {
    console.log(source.raw_file_contents.trim());
  }
} finally {
  rclnodejs.shutdown();
}
```

Run the query:

```bash
node describe-type.mjs
```

It prints `std_msgs/msg/String`, `Fields: data`, and the source when available. The program waits separately for the service and publisher discovery, then sends one request with a timeout. It selects the publisher by node name and namespace rather than assuming the first endpoint belongs to the target node.

Use the hash reported by the endpoint, not a hash made from the type name. A type hash identifies a particular interface definition. The service does not provide an inventory of every type on the ROS graph: the target node has to know the requested type.

## Read the Response

Check `successful` before reading the description. If it is false, `failure_reason` explains why the node could not answer. Check the fully qualified type name, the hash, and whether the target node uses that type.

The outer `type_description` contains the requested definition and `referenced_type_descriptions` for nested types. You do not need a new request for each referenced field. Set `include_type_sources` to `false` if you only need the field definitions.

## Control Startup

rclnodejs starts the service by default on supported ROS versions. Check the startup setting on the running publisher:

```bash
ros2 param get /talker start_type_description_service
```

The parameter is read-only once declared. To stop the publisher from answering type queries, restart it with:

```bash
node publisher.mjs --ros-args -p start_type_description_service:=false
```

With this override, the service can still appear in the ROS graph even though the node does not answer requests. To omit the endpoint entirely, pass `{ startTypeDescriptionService: false }` as the fourth argument to `createNode()`, after the name, namespace, and context.

If the service is missing, verify the ROS version, node namespace, and startup configuration. A service appearing in the graph does not mean it will answer unless the target node is spinning.

Press **Ctrl+C** to stop the publisher.

## Further Reading

- [ROS 2 interfaces](https://docs.ros.org/en/lyrical/ROS-Framework/interfaces/Working-with-interfaces.html) introduces message and service definitions.
- [Type-description tests](../test/test-type-description-service.js) cover the request and startup behavior.
