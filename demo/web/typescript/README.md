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

The action panel needs a package build containing the action SDK. To try
the current repository code before it is published, build it at the
repository root, then use it without changing the demo's dependency or lockfile:

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

`server.ts` runs the runtime *plus* a tiny `/add_two_ints` service and a
1 Hz `/web_demo_tick` publisher, plus a bounded, cancellable Fibonacci
action using `example_interfaces/action/Fibonacci`.

> The HTTP transport here serves `call`, `publish`, and SSE actions; `subscribe`
> uses WebSocket. HTTP `subscribe` over Server-Sent Events is an opt-in
> (`new HttpTransport({ sse: true })`, or `--http-sse` on the CLI) — see
> the [JavaScript demo](../javascript/README.md) for a working SSE +
> `EventSource` example.

**Shell 2 — Vite dev server:**

```bash
npm run dev
# ➜  Local:  http://localhost:8080/
```

For other ports, set `RUNTIME_PORT` and `HTTP_PORT` on the runtime and
run `npm run dev -- --port 8081`. Open the page with
`?wsPort=9010&httpPort=9011` to select matching runtime ports.

## Fibonacci actions

The action panel accepts orders from 2 to 12, sends feedback every
half-second, and displays the final result and terminal status.
The goal, feedback, and result types are derived from the ROS action name:

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

The type-only import loads ROS declarations without adding the native
addon to browser JavaScript. HTTP actions use a dedicated `{ http }`
client; the demo's general `{ http, ws }` client still supports topic
subscriptions on a different port.

- **WebSocket:** **Cancel Goal** requests cancellation and displays the server's partial result and final status.
- **HTTP:** **Cancel Goal** is disabled. **Stop Streaming** closes the client stream without canceling the ROS goal.
- Changing transports or leaving the page closes the current action client; stale feedback cannot overwrite later goals.

The runtime enables CORS for this local cross-origin demo. Restrict it
for production. Actions use POST with `fetch()` streaming, not the
GET-only `EventSource`; the `sse` option only enables HTTP subscriptions.
Feedback and acceptance can arrive in either order. A resolved result
can also be canceled or aborted, so inspect `goal.status`.

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

The action panel additionally needs a ROS action server at `/fibonacci`
using `example_interfaces/action/Fibonacci` when the bundled server is
not used. The CLI does not create the sample ROS nodes.

## OpenAPI export

With ROS 2 sourced, export the same allow-list, including the action:

```bash
npx rclnodejs-web openapi web.json > openapi.json
```

The action response is an SSE stream. Its schemas describe individual
event payloads, not one JSON response, and API explorers may buffer it.

## Other npm scripts

| Command             | What it does                                         |
| ------------------- | ---------------------------------------------------- |
| `npm run typecheck` | `tsc --noEmit` — silent on success                   |
| `npm run build`     | static bundle in `dist/`                             |
| `npm run preview`   | serve the built `dist/`                              |
