# rosocket — ROS 2 in the browser, no library required

> A tiny WebSocket gateway to ROS 2 — built into `rclnodejs`.

> **Availability:** experimental; currently only on the `develop` branch of
> `rclnodejs` and not yet part of any published release. Install from GitHub
> to try it (see the project's [Install from GitHub](../README.md#install-from-github) section):
>
> ```bash
> npm install RobotWebTools/rclnodejs#develop
> ```

**rosocket** is a **lightweight** WebSocket bridge that lets a **plain web
browser** (or any WebSocket-capable client) talk to ROS 2 through `rclnodejs`,
with **no extra JavaScript library** required on the client side. Browsers
only need the built-in `WebSocket` and `JSON` APIs.

How it compares with the classic
[rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite) +
[roslibjs](https://github.com/RobotWebTools/roslibjs) stack:

| | **rosocket (rclnodejs)** | **rosbridge_suite + roslibjs** |
| --- | --- | --- |
| Server process | same Node.js process as your `rclnodejs` app | separate Python ROS 2 node |
| Client-side library | none — built-in `WebSocket` + `JSON` | `roslibjs` (must be bundled/loaded) |
| Wire protocol | resource-style URLs (`/topic/<name>`, `/service/<name>`); frame = bare ROS message as JSON | custom JSON envelope (`op: "publish" / "subscribe" / "call_service"`, …) |
| Type discovery | URL `?type=` query, or server-side default map | advertised at runtime via envelope ops |
| Features | publish / subscribe, service client | pub/sub, services, **actions, tf, parameters, compression, PNG/CBOR, auth, …** |
| Deployment | one `npm` dep, runs anywhere Node runs | extra ROS package; version must match ROS distro |

## URL scheme

The bridge is **resource-style** — the URL *is* the topic or service name and
the WebSocket frame *is* the ROS message as JSON.

| URL | Direction | Payload |
| --- | --- | --- |
| `ws://host:port/topic/<topic_name>?type=<pkg>/msg/<Type>` | server → client (subscribe) | one frame per received ROS message, JSON-serialized |
| `ws://host:port/topic/<topic_name>?type=<pkg>/msg/<Type>` | client → server (publish) | one frame per ROS message to publish, JSON-encoded |
| `ws://host:port/service/<service_name>?type=<pkg>/srv/<Type>` | client → server (request) | one frame per request, JSON-encoded |
| `ws://host:port/service/<service_name>?type=<pkg>/srv/<Type>` | server → client (response) | one frame per response, JSON-serialized |

Notes:

- Each connection is dedicated to one topic or service. A single socket is
  full-duplex, so the same `/topic/<name>` socket can both publish and
  subscribe at the same time.
- The `type=` query parameter can be omitted if the server was started with
  `topicTypes` / `serviceTypes` defaults for that name.
- Service calls may be sent as a bare request (`{"a":1,"b":2}`) or wrapped
  with a correlation id (`{"id":"c1","request":{"a":1,"b":2}}`); responses
  echo the same shape (`{"id":"c1","response":{...}}`).
- Errors are reported as `{"error":"<message>"}` frames; fatal protocol errors
  cause the socket to close with a `1008`/`1011` code.
- 64-bit integer fields may be sent as JSON numbers or BigInt-encoded
  strings (`"12n"`); responses use the rclnodejs `toJSONSafe` encoding
  (BigInts become `"<n>n"` strings).

## Server side

```js
const rclnodejs = require('rclnodejs');
const { startRosocket } = require('rclnodejs/rosocket');

await rclnodejs.init();
const node = new rclnodejs.Node('rosocket_node');
rclnodejs.spin(node);

await startRosocket({
  node,
  port: 9000,
  // optional: pre-declare types so clients can omit ?type=
  topicTypes:   { '/chatter':       'std_msgs/msg/String' },
  serviceTypes: { '/add_two_ints':  'example_interfaces/srv/AddTwoInts' },
});
```

### Without `topicTypes` / `serviceTypes`

The `topicTypes` / `serviceTypes` maps are entirely optional. If you omit
them, the server stays generic and clients must specify the message type
themselves via the `?type=` query parameter on each connection:

```js
// server – open to any topic/service the node is allowed to access
await startRosocket({ node, port: 9000 });
```

```js
// browser – type comes from the URL
const sub = new WebSocket(
  'ws://localhost:9000/topic/chatter?type=std_msgs/msg/String'
);
const cli = new WebSocket(
  'ws://localhost:9000/service/add_two_ints?type=example_interfaces/srv/AddTwoInts'
);
```

The same applies to the CLI — drop `--topic` / `--service` to run a generic
bridge: `npx rosocket --port 9000`.

## CLI (`rosocket`)

A ready-to-run command is shipped as a `bin` entry, so users do not need to
write any server code:

```bash
# from inside this repo
npm run rosocket -- --port 9000 \
  --topic   /chatter:std_msgs/msg/String \
  --service /add_two_ints:example_interfaces/srv/AddTwoInts

# anywhere after `npm i rclnodejs` (or via npx)
npx rosocket --port 9000 \
  --topic   /chatter:std_msgs/msg/String \
  --service /add_two_ints:example_interfaces/srv/AddTwoInts
```

Options: `--port/-p`, `--host/-H`, `--node-name/-n`, repeatable
`--topic/-t <name>:<type>` and `--service/-s <name>:<type>`, `--help/-h`.
Pre-declared types let browsers omit the `?type=` query.

## Browser side (no library)

```html
<script type="module">
  // Subscribe
  const sub = new WebSocket('ws://localhost:9000/topic/chatter');
  sub.onmessage = (e) => console.log('chatter:', JSON.parse(e.data).data);

  // Publish on the same socket (or a different one)
  sub.onopen = () => sub.send(JSON.stringify({ data: 'hello from browser' }));

  // Service call
  const cli = new WebSocket('ws://localhost:9000/service/add_two_ints');
  cli.onopen    = () => cli.send(JSON.stringify({ a: 1, b: 2 }));
  cli.onmessage = (e) => console.log('sum =', JSON.parse(e.data).sum);
</script>
```

## Why not rosbridge?

Use this bridge when you want:

- **Zero browser dependency** — no JavaScript library to bundle or load.
- **Zero extra process** — already in the same Node.js where your
  `rclnodejs` app runs.
- **Greppable URLs** for reverse-proxy ACLs (`location /topic/...`).

Use a full-featured stack like rosbridge_suite when you need actions, tf,
parameter helpers, compression, throttling, or compatibility with existing
ROS web tooling.
