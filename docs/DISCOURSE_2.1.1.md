**rclnodejs 2.1.1: SSE Support for Streaming Robot Data to the Web**

[rclnodejs 2.1.1](https://github.com/RobotWebTools/rclnodejs/releases/tag/2.1.1) is out, adding Server-Sent Events (SSE) `subscribe` support to the web-native HTTP transport described above. Browsers can now stream live ROS 2 topic data with just the built-in `EventSource` API — no WebSocket, no SDK, no codegen.

Builds on 2.1.0's native ESM and the `rclnodejs/web` HTTP transport from 2.0.0: same capability runtime, now with a `subscribe` path over SSE alongside the existing `call`/`publish`.

## Streaming topics into the browser

```bash
# Terminal 1
ros2 run demo_nodes_cpp talker  # publishes /chatter
```

```bash
# Terminal 2
npx -p rclnodejs rclnodejs-web --port 9000 --http-port 9001 \
  --http-sse --http-cors '*' --subscribe /chatter=std_msgs/msg/String
```

```js
// Browser — no SDK needed
const source = new EventSource(
  'http://localhost:9001/capability/subscribe/chatter'
);
source.addEventListener('message', (e) => console.log(JSON.parse(e.data)));
```

`GET /capability/subscribe/<name>` streams as `text/event-stream`; `--http-cors` lets cross-origin pages connect too, and `--http-sse-keep-alive <ms>` controls the heartbeat interval. Full walkthrough: [`demo/web/javascript/`](https://github.com/RobotWebTools/rclnodejs/tree/develop/demo/web/javascript).

Useful anywhere you want live robot data outside a ROS-aware process — a browser dashboard showing topic data in real time, or an agent/LLM tool that subscribes to a topic over plain HTTP instead of speaking ROS 2 natively.

Feedback welcome.

Cheers,
Minggang
