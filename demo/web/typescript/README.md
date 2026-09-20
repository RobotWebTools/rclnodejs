# End-to-end typed ROS 2 in a TypeScript app

A **TypeScript** browser entry built with **Vite**, driven by the typed
SDK so request, reply, message, and action shapes are visible in your IDE.

```ts
const reply = await ros.call<'example_interfaces/srv/AddTwoInts'>(
  '/add_two_ints', { a: '2n', b: '40n' }
);
// reply.sum is typed as `${number}n` — no hand-written types, no codegen.
```

## Run it (two shells)

```bash
source /opt/ros/<distro>/setup.bash
cd demo/web/typescript
npm install
```

Until a released package includes the action SDK, use the checkout instead
without changing the dependency or lockfile:

```bash
# From the repository root, with ROS 2 sourced and npm install completed:
npm run build:dist
cd demo/web/typescript
npm install --no-save --package-lock=false --ignore-scripts ../../..
```

**Shell 1 — runtime:**

```bash
source /opt/ros/<distro>/setup.bash
npm run server
# rclnodejs/web : ws://localhost:9000/capability
#               also http://localhost:9001/capability
```

`server.ts` includes `/add_two_ints`, a 1 Hz `/web_demo_tick` publisher,
and a cancellable `/fibonacci` server (`example_interfaces/action/Fibonacci`).

> HTTP serves calls, publishes, and SSE actions; subscriptions use WebSocket.
> For HTTP topic subscriptions, see the [JavaScript demo](../javascript/README.md).

**Shell 2 — Vite dev server:**

```bash
npm run dev
# ➜  Local:  http://localhost:8080/
```

For custom ports, set `RUNTIME_PORT`/`HTTP_PORT`, match the page's
`?wsPort=9010&httpPort=9011`, and use Vite's `--port` option.

## Fibonacci actions

Send orders from 2 to 12 and receive feedback every half-second. The ROS
action name supplies goal, feedback, and result types:

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

The type-only import supplies ROS declarations without a native browser
dependency. A dedicated `{ http }` client streams actions; subscriptions
stay on WebSocket.

- WebSocket: **Cancel Goal** requests cancellation.
- HTTP/SSE: **Stop Streaming** disconnects without canceling the ROS goal.
- Switching transports or leaving the page closes the client and ignores late events.

Check `goal.status` even when `goal.result` resolves. Wildcard CORS is for
local testing only. Actions use `fetch()` streaming, not `EventSource`;
the `sse` option only enables HTTP subscriptions.

```bash
curl --fail-with-body -sS -N http://localhost:9001/capability/action/fibonacci \
  -H 'content-type: application/json' -d '{"order":3}'
```

## Without the bundled `server.ts`

`npm run server` bundles the runtime and the demo's sample nodes into one
process so it runs out of the box. In a real project those nodes already
run elsewhere, so you only need the runtime — replace shell 1 with the
CLI (shell 2 and `src/main.ts` are unchanged):

```bash
npx rclnodejs-web web.json

# plus the nodes the demo expects:
ros2 run demo_nodes_cpp add_two_ints_server
# (and any std_msgs/String publisher on /web_demo_tick)
```

For CLI mode, also run a `/fibonacci` server using
`example_interfaces/action/Fibonacci`.

## OpenAPI export

With ROS 2 sourced, export the same allow-list, including the action:

```bash
npx rclnodejs-web openapi web.json > openapi.json
```

SSE schemas describe per-event data; API explorers may buffer the stream.

## Other npm scripts

| Command             | What it does                                         |
| ------------------- | ---------------------------------------------------- |
| `npm run typecheck` | `tsc --noEmit` — silent on success                   |
| `npm run build`     | static bundle in `dist/`                             |
| `npm run preview`   | serve the built `dist/`                              |
