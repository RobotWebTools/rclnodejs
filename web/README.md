# rclnodejs/web — Browser SDK guide

> Talk to ROS 2 from a web app — typed, allow-listed, `curl`-able,
> OpenAPI-documented.

`rclnodejs/web` is the browser-side of `rclnodejs`: a compact ESM
module plus a server runtime that together expose a declarative
subset of your ROS 2 graph over WebSocket **and** plain HTTP. The
browser API has four verbs: `call`, `publish`, `subscribe`, and `action`, typed
end-to-end from your ROS 2 message, service, and action types. The same
`expose` config also generates an OpenAPI 3.1 document, so
codegen, API explorers, and AI-agent tool-use all get a standard,
machine-readable description of your ROS 2 graph for free.

For runnable code see [`demo/web/`](../demo/web/):

| Demo                                              | Pick this if you…                                                           |
| ------------------------------------------------- | --------------------------------------------------------------------------- |
| [`demo/web/javascript/`](../demo/web/javascript/) | want a single static page — no build tools, no `npm install` for the page   |
| [`demo/web/typescript/`](../demo/web/typescript/) | already have a Vite / Next / React / Vue / Svelte project, want full typing |

For HTTP/SSE actions, see the [Fibonacci browser demo](../demo/web/javascript/README.md#fibonacci-actions).

## 1. Server side: stand up the runtime

> `-p rclnodejs` tells npx the `rclnodejs-web` binary lives inside the
> `rclnodejs` package; drop it once `rclnodejs` is already installed in
> the current project.

```bash
source /opt/ros/<distro>/setup.bash
npx -p rclnodejs rclnodejs-web \
  --port 9000 --http-port 9001 \
  --call /add_two_ints=example_interfaces/srv/AddTwoInts \
  --publish /chatter=std_msgs/msg/String \
  --subscribe /scan=sensor_msgs/msg/LaserScan \
  --action /fibonacci=test_msgs/action/Fibonacci
# rclnodejs/web listening on ws://localhost:9000/capability (4 capabilities)
#                also http://localhost:9001/capability (call/publish/action)
```

Or feed the same allow-list from `web.json`:

```json
{
  "port": 9000,
  "http": { "port": 9001 },
  "expose": {
    "call": { "/add_two_ints": "example_interfaces/srv/AddTwoInts" },
    "publish": { "/chatter": "std_msgs/msg/String" },
    "subscribe": { "/scan": "sensor_msgs/msg/LaserScan" },
    "action": { "/fibonacci": "test_msgs/action/Fibonacci" }
  }
}
```

```bash
npx -p rclnodejs rclnodejs-web web.json
```

> The `expose` block is the **public API** your browser depends on.
> Anything not listed is rejected with `code: 'not_exposed'` before
> any ROS 2 API runs. Keep it narrow.

## 2. Client side: talk to it from the browser

### Connect

```ts
import type {} from 'rclnodejs';
import { connect } from 'rclnodejs/web'; // or via esm.sh in a <script type="module">
```

The type-only import loads ROS declarations and is erased at runtime.
Omit it in JavaScript.

`connect()` accepts three URL shapes — the SDK picks transport(s)
from the scheme:

| You want…                          | Pass                                                            |
| ---------------------------------- | --------------------------------------------------------------- |
| WebSocket only                     | `'ws://host:9000/capability'`                                   |
| HTTP + WS behind one reverse proxy | `'http://host:9001'`                                            |
| HTTP + WS on different ports       | `{ http: 'http://host:9001', ws: 'ws://host:9000/capability' }` |
| HTTP only (no `subscribe()`)       | `{ http: 'http://host:9001' }`                                  |

A bare `http://` URL auto-derives the WS sibling at the same origin
(`/capability` path); the `{ http }`-only form disables WS entirely
and `subscribe()` rejects with `transport_unavailable`.

Actions use SSE with HTTP-only endpoints and WebSocket with an explicit
`{ http, ws }` pair.

```ts
const ros = await connect({
  http: 'http://localhost:9001',
  ws: 'ws://localhost:9000/capability',
});
```

### The verb API

The snippet below is **TypeScript** — the `<'pkg/.../Type'>` generic
in angle brackets is what drives end-to-end typing of the payload
and reply from your ROS 2 message types (no codegen, no
shared types module). From plain JavaScript, drop the generic and
the calls behave identically.

```ts
// Service call — '7n' / '35n' are the string forms of BigInt 7n / 35n;
// ROS 2 64-bit integers round-trip as strings to survive JSON.
const reply = await ros.call<'example_interfaces/srv/AddTwoInts'>(
  '/add_two_ints',
  { a: '7n', b: '35n' }
);
reply.sum; // typed as `${number}n`, runtime value '42n'

// Publish — resolves to undefined on success
await ros.publish<'std_msgs/msg/String'>('/chatter', { data: 'hello' });

// Subscribe — always uses WebSocket
const sub = await ros.subscribe<'std_msgs/msg/String'>('/chatter', (msg) =>
  console.log(msg.data)
);
await sub.close();
```

### Actions

Start and expose the ROS action server before sending a goal.

```ts
const httpClient = await connect({ http: 'http://localhost:9001' });
try {
  const goal = await httpClient.action<'test_msgs/action/Fibonacci'>(
    '/fibonacci',
    { order: 5 },
    { onFeedback: (feedback) => console.log(feedback.sequence) }
  );
  const result = await goal.result;
  console.log(goal.status, result.sequence);
} catch (error) {
  console.error(error);
} finally {
  await httpClient.close();
}
```

HTTP actions use POST/SSE via `fetch()`; `EventSource` cannot POST goals.
`--http-sse` is only needed for subscriptions. Feedback may precede `accepted`.
Request errors reject `action()`; stream errors reject `goal.result`.
A resolved result may be canceled or aborted, so check `goal.status`.

For cancellation, submit the goal over WebSocket:

```ts
const wsClient = await connect({
  http: 'http://localhost:9001',
  ws: 'ws://localhost:9000/capability',
});
try {
  const goal = await wsClient.action<'test_msgs/action/Fibonacci'>(
    '/fibonacci',
    { order: 10 },
    { onFeedback: (feedback) => console.log(feedback.sequence) }
  );
  await goal.cancel();
  const result = await goal.result;
  console.log(goal.status, result.sequence);
} finally {
  await wsClient.close();
}
```

Cancellation depends on server acceptance. HTTP `cancel()` rejects with
`unsupported_kind`; closing an HTTP stream does not cancel the goal.

### Lifecycle and cleanup

Each `subscribe()` returns a handle with its own `close()`; the
top-level `ros.close()` cancels every active subscription and shuts
down both transports. Pending HTTP actions reject with `connection_lost`;
the ROS goals are not canceled.

```ts
const sub = await ros.subscribe('/chatter', handler);
// …
sub.close(); // drop just this subscription
await ros.close(); // tear down the whole connection

// Typical browser cleanup:
window.addEventListener('beforeunload', () => ros.close());
```

## 3. curl recipes (no JavaScript at all)

When `--http-port` is on, every `call` / `publish` / `action` is reachable from
any HTTP client — curl, Postman, AI-agent tool-use, no SDK required.
With `--http-sse` (or `"http": { "sse": true }`), `subscribe` is also
reachable over HTTP as a Server-Sent Events stream.

```bash
# Service call
curl -sS -X POST http://localhost:9001/capability/call/add_two_ints \
  -H 'content-type: application/json' \
  -d '{"a":"7n","b":"35n"}'
# => {"sum":"42n"}

# Publish (returns 204 No Content)
curl -sS -X POST http://localhost:9001/capability/publish/chatter \
  -H 'content-type: application/json' \
  -d '{"data":"hi from curl"}'

# Action (requires the Fibonacci server; streams feedback and a final result)
curl --fail-with-body -sS -N http://localhost:9001/capability/action/fibonacci \
  -H 'content-type: application/json' \
  -d '{"order":3}'

# Subscribe over Server-Sent Events (needs --http-sse). Streams until
# you disconnect; -N keeps curl from buffering the event stream.
curl -N http://localhost:9001/capability/subscribe/chatter
# event: ready
# data: {"capability":"/chatter","subId":"sse"}
#
# event: message
# data: {"data":"hello"}
```

From a browser, the same SSE endpoint works with the built-in
`EventSource` (cross-origin when `--http-cors` is set):

```js
const es = new EventSource(
  'http://localhost:9001/capability/subscribe/chatter'
);
es.addEventListener('message', (e) => {
  const msg = JSON.parse(e.data);
  console.log(msg.data);
});
es.addEventListener('error', () => es.close());
```

### OpenAPI export

Want the same HTTP surface as a browsable/machine-readable spec
instead of hand-writing routes? `rclnodejs-web openapi` prints an
OpenAPI 3.1 document for the same `expose` config, without starting
the runtime:

```bash
npx -p rclnodejs rclnodejs-web openapi web.json > openapi.json
```

SSE response schemas describe per-event payloads. Use an SSE-aware client for
live feedback; API explorers may buffer responses.

See [`demo/web/javascript/`](../demo/web/javascript/) for a full
walkthrough, including browsing it in Swagger UI.

## 4. `rclnodejs/web` vs. `rosbridge` + `roslibjs`

`rosbridge` + `roslibjs` is the standard browser-side ROS 2 stack of the
past decade. Both stacks target the same job (talk to ROS 2 from a web
app over WebSocket + JSON) and both keep the browser facing
topics/services rather than inventing a higher-level abstraction. What
differs is **what's exposed to the browser, how strongly it's typed,
and whether plain HTTP works**:

|                             | **`rclnodejs/web`**                                                  | `rosbridge` + `roslibjs`          |
| --------------------------- | -------------------------------------------------------------------- | --------------------------------- |
| **Public API surface**      | **`web.json` allow-list — reviewable artifact**                      | The whole live ROS graph          |
| **TypeScript types**        | One ROS 2 type name → fully typed request/response/message | `any`; bolt-on community packages |
| **HTTP `call` / `publish`** | ✅ — `curl`, Postman, AI-agent tool-use just work                    | ❌ (WebSocket only)               |
