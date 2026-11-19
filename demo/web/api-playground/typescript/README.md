# API Playground - TypeScript

This Vite app uses the [`rclnodejs/web` SDK](../../../../web/README.md) to work with ROS 2 topics, services, and actions in TypeScript.

```ts
const reply = await ros.call<'example_interfaces/srv/AddTwoInts'>(
  '/add_two_ints',
  { a: '2n', b: '40n' }
);
// The SDK types reply.sum as `${number}n` using the generated declarations.
```

## Run it (two shells)

Use a supported version of Node.js and ROS 2 with `example_interfaces` and `std_msgs` installed. From the repository root, run these commands. Replace `lyrical` with your distro:

```bash
source /opt/ros/lyrical/setup.bash
cd demo/web/api-playground/typescript
npm install
```

The published rclnodejs v2.3.1 package includes the action SDK. If you have an older install, run `npm update rclnodejs` in the demo directory.

Start the runtime from this directory with ROS sourced:

```bash
npm run server
# rclnodejs/web : ws://localhost:9000/capability
#               also http://localhost:9001/capability
```

The runtime starts `/add_two_ints`, publishes `/web_demo_tick` at 1 Hz, and runs a cancellable `/fibonacci` action (`example_interfaces/action/Fibonacci`).

This demo subscribes over WebSocket. See the [JavaScript playground](../javascript/README.md) for HTTP subscriptions.

In a second terminal in the demo directory, start Vite:

```bash
npm run dev
# ➜  Local:  http://localhost:8080/
```

Open http://localhost:8080/. To change the runtime ports, set `RUNTIME_PORT` and `HTTP_PORT`, then match them in the page URL, for example `?wsPort=9010&httpPort=9011`. Use Vite's `--port` option to change the page port.

## Fibonacci actions

Send an order from 2 to 12. The server sends feedback every half-second. Pass the ROS action type to type the goal, feedback, and result:

```ts
import type {} from 'rclnodejs';
import { connect } from 'rclnodejs/web';

const client = await connect({ http: 'http://localhost:9001' });
try {
  const goal = await client.action<'example_interfaces/action/Fibonacci'>(
    '/fibonacci',
    { order: 5 },
    { onFeedback: (feedback) => console.log(feedback.sequence) }
  );
  const result = await goal.result;
  console.log(goal.status, result.sequence);
} finally {
  await client.close();
}
```

`import type` makes the generated declarations available without loading the native addon in the browser. The `{ http }` client streams action results.

- Over WebSocket, **Cancel Goal** requests cancellation.
- Over HTTP/SSE, **Stop Streaming** disconnects without canceling the goal.

The page closes the client and ignores late events when you switch transports or leave. Check `goal.status` even when `goal.result` resolves.

HTTP actions stream with `fetch()`, not `EventSource`. The `sse` option enables HTTP topic subscriptions. Use wildcard CORS only for local testing.

```bash
curl --fail-with-body -sS -N http://localhost:9001/capability/action/fibonacci \
  -H 'content-type: application/json' -d '{"order":3}'
```

## Without the bundled `server.ts`

To use your own ROS nodes, run the CLI instead of `npm run server`. Keep Vite running. Start the CLI and the ROS service in separate ROS-sourced terminals:

```bash
npx rclnodejs-web web.json

# plus the nodes the demo expects:
ros2 run demo_nodes_cpp add_two_ints_server
# (and any std_msgs/msg/String publisher on /web_demo_tick)
```

Also run `/fibonacci` with the `example_interfaces/action/Fibonacci` type.

## OpenAPI export

With ROS sourced, export the config, including the action:

```bash
npx rclnodejs-web openapi web.json > openapi.json
```

SSE schemas describe one event at a time. Some API explorers buffer the stream.

## Other npm scripts

`npm run typecheck` checks the types without building. `npm run build` writes the app to `dist/`, and `npm run preview` serves that build.
