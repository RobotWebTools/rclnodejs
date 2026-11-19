# rosocket

rosocket exposes ROS 2 topics and services over WebSocket URLs. It runs in your `rclnodejs` process; browser clients use the built-in `WebSocket` and `JSON` APIs.

Install [rclnodejs](../README.md) with a sourced ROS 2 environment:

```bash
npm install rclnodejs
```

For actions, typed browser clients, or a server-side allow-list, use [rclnodejs/web](../web/README.md).

## URL scheme

| URL | Direction | Payload |
| --- | --- | --- |
| `ws://host:port/topic/<topic_name>?type=<pkg>/msg/<Type>` | server to client (subscribe) | one frame per received message |
| `ws://host:port/topic/<topic_name>?type=<pkg>/msg/<Type>` | client to server (publish) | one frame per message to publish |
| `ws://host:port/service/<service_name>?type=<pkg>/srv/<Type>` | client to server (request) | one frame per request |
| `ws://host:port/service/<service_name>?type=<pkg>/srv/<Type>` | server to client (response) | one frame per response |

- Each connection uses one topic or service. A topic socket can both publish and subscribe.
- Omit `type=` when the server has a `topicTypes` or `serviceTypes` entry for that name.
- Send service requests directly (`{"a":"1n","b":"2n"}`) or with an ID (`{"id":"c1","request":{"a":"1n","b":"2n"}}`). Wrapped replies use `{"id":"c1","response":{...}}`.
- Errors arrive as `{"error":"<message>"}` frames. Fatal protocol errors close the socket with code `1008` or `1011`.
- Encode 64-bit integer fields as strings such as `"12n"`, not JSON numbers. Responses use the same encoding for BigInts.

## Server side

Save as `server.mjs` and run with `node server.mjs`:

```js
import rclnodejs from 'rclnodejs';
import { startRosocket } from 'rclnodejs/rosocket';

await rclnodejs.init();
const node = new rclnodejs.Node('rosocket_node');
rclnodejs.spin(node);

const bridge = await startRosocket({
  node,
  host: '127.0.0.1',
  port: 9000,
  topicTypes: { '/chatter': 'std_msgs/msg/String' },
  serviceTypes: { '/add_two_ints': 'example_interfaces/srv/AddTwoInts' },
});
```

Stop the gateway with `await bridge.close()`, then call `rclnodejs.shutdown()`.

### Without type maps

Without type maps, clients must include the type in the URL:

```js
await startRosocket({ node, host: '127.0.0.1', port: 9000 });
```

```js
const sub = new WebSocket(
  'ws://127.0.0.1:9000/topic/chatter?type=std_msgs/msg/String'
);
const cli = new WebSocket(
  'ws://127.0.0.1:9000/service/add_two_ints?type=example_interfaces/srv/AddTwoInts'
);
```

For the CLI, omit `--topic` and `--service` to require types in client URLs.

## CLI (`rosocket`)

From the repository root:

```bash
npm run rosocket -- --host 127.0.0.1 --port 9000 \
  --topic   /chatter:std_msgs/msg/String \
  --service /add_two_ints:example_interfaces/srv/AddTwoInts
```

After installing the package:

```bash
npx rosocket --host 127.0.0.1 --port 9000 \
  --topic   /chatter:std_msgs/msg/String \
  --service /add_two_ints:example_interfaces/srv/AddTwoInts
```

Options: `--port/-p`, `--host/-H`, `--node-name/-n`, repeatable `--topic/-t <name>:<type>` and `--service/-s <name>:<type>`, `--help/-h`.

## Browser side (no library)

Run the server above and a ROS `/add_two_ints` service of type `example_interfaces/srv/AddTwoInts`.

```html
<script type="module">
  // Subscribe
  const sub = new WebSocket('ws://127.0.0.1:9000/topic/chatter');
  sub.onmessage = (e) => console.log('chatter:', JSON.parse(e.data).data);

  // Publish on the same socket (or a different one)
  sub.onopen = () => sub.send(JSON.stringify({ data: 'hello from browser' }));

  // Service call
  const cli = new WebSocket('ws://127.0.0.1:9000/service/add_two_ints');
  cli.onopen = () => cli.send(JSON.stringify({ a: '1n', b: '2n' }));
  cli.onmessage = (e) => console.log('sum =', JSON.parse(e.data).sum);
</script>
```

## Access control

The bridge listens on `0.0.0.0` by default; the examples bind to loopback. Type maps supply defaults, not an allow-list: clients can select other resources and types through their URLs. Use the `verifyClient` hook or an authenticated reverse proxy before exposing the gateway to a network.

For tf, parameter helpers, compression, or throttling, see [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite) and [roslibjs](https://github.com/RobotWebTools/roslibjs).
