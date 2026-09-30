# rclnodejs/web — Browser SDK guide

> Talk to ROS 2 from a web app — typed, allow-listed, `curl`-able,
> OpenAPI-documented.

`rclnodejs/web` provides a compact ESM browser SDK. Together with the
`rclnodejs/web/server` Node.js runtime, it exposes a declarative
subset of your ROS 2 graph over WebSocket **and** plain HTTP. The
browser API has four verbs: `call`, `publish`, `subscribe`, and `action`, typed
end-to-end from your ROS 2 message, service, and action types. The same
`expose` config can be exported as an OpenAPI 3.1 document for codegen,
API explorers and HTTP tool-use. It describes the **exposed API**, not
the entire ROS graph; generated types and schemas do not imply runtime
schema enforcement.

For runnable code see [`demo/web/`](../demo/web/):

| Demo                                              | Pick this if you…                                                           |
| ------------------------------------------------- | --------------------------------------------------------------------------- |
| [`demo/web/javascript/`](../demo/web/javascript/) | want a single static page — no build tools, no `npm install` for the page   |
| [`demo/web/typescript/`](../demo/web/typescript/) | already have a Vite / Next / React / Vue / Svelte project, want full typing |

## Functionality at a glance

The table maps ROS 2 communication features to the roles a web client can
perform and the web transport each role supports. The action rows below
describe the **2.3.0 implementation**; use a checkout or package that includes
it (see the [TypeScript demo setup](../demo/web/typescript/README.md#run-it-two-shells)).
This is an implementation matrix, not a claim that every item in the
[web runtime roadmap](../docs/WEB_RUNTIME_ROADMAP.md) has shipped.

### ROS 2 roles and transport support

All supported operations must be in the corresponding `expose` allow-list.
HTTP paths below use the default `/capability` base path. The roles describe
the **web client's participation**, not the complete native rclnodejs API.

| ROS 2 feature / web-client role                     | WebSocket                                   | HTTP / SSE                                                                                  | Web SDK / scope                                                                                                              |
| --------------------------------------------------- | ------------------------------------------- | ------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------- |
| Pub/Sub: topic publisher                            | Supported                                   | HTTP `POST /capability/publish/<name>`; 204 on success                                      | `ros.publish(name, message)` publishes into ROS.                                                                             |
| Pub/Sub: topic subscriber                           | Supported; topics share one connection      | HTTP `GET /capability/subscribe/<name>` with SSE; requires `--http-sse` or `http.sse: true` | `ros.subscribe()` uses **WS only**. HTTP consumers use `EventSource` or curl.                                                |
| Client/Service: service client                      | Supported                                   | HTTP `POST /capability/call/<name>`; JSON response                                          | `ros.call()` invokes a ROS service, including servers created with native `node.createService()`.                            |
| Client/Service: browser-hosted service server       | Not exposed                                 | Not exposed                                                                                 | Reverse RPC (ROS requests to browser handlers, correlated responses and handler disconnect handling) is not implemented.     |
| Action client: send goals, receive feedback/results | Supported                                   | HTTP `POST /capability/action/<name>` with SSE; **no `--http-sse` required**                | `ros.action(name, goal, { onFeedback })`; await `goal.result`, then inspect `goal.status`.                                   |
| Action: client cancellation                         | Supported, subject to ROS server acceptance | **Not supported**                                                                           | `goal.cancel()` requires a WS action handle; HTTP handles reject with `unsupported_kind`.                                    |
| Action: browser-hosted action server                | Not exposed                                 | Not exposed                                                                                 | Browser-side goal acceptance/execution, feedback/results and cancellation/disconnect lifecycle handling are not implemented. |

### Protocol notes and scope

These protocols connect web clients to the runtime; service/action servers
remain on the native ROS side. HTTP actions use POST/fetch streaming;
disconnecting does **not** cancel the ROS goal.

See [connection options](#connect), [SDK operations](#the-verb-api),
[actions](#actions) and [curl/SSE recipes](#3-curl-recipes-no-javascript-at-all)
for transport details and examples.

## 1. Server side: stand up the runtime

> `-p rclnodejs` tells npx the `rclnodejs-web` binary lives inside the
> `rclnodejs` package; drop it once `rclnodejs` is already installed in
> the current project. For a source checkout, run `node bin/rclnodejs-web.js`
> from the repository root instead of the npx prefix below.

```bash
source /opt/ros/<distro>/setup.bash
npx -p rclnodejs rclnodejs-web \
  --port 9000 --http-port 9001 \
  --http-sse --http-cors http://localhost:8080 \
  --call /add_two_ints=example_interfaces/srv/AddTwoInts \
  --publish /chatter=std_msgs/msg/String \
  --subscribe /chatter=std_msgs/msg/String \
  --action /fibonacci=example_interfaces/action/Fibonacci
# rclnodejs/web listening on ws://localhost:9000/capability (4 capabilities)
#                also http://localhost:9001/capability (call/publish/action + subscribe (SSE))
```

Or feed the same allow-list from `web.json`:

```json
{
  "port": 9000,
  "http": {
    "port": 9001,
    "sse": true,
    "cors": "http://localhost:8080"
  },
  "expose": {
    "call": { "/add_two_ints": "example_interfaces/srv/AddTwoInts" },
    "publish": { "/chatter": "std_msgs/msg/String" },
    "subscribe": { "/chatter": "std_msgs/msg/String" },
    "action": { "/fibonacci": "example_interfaces/action/Fibonacci" }
  }
}
```

```bash
npx -p rclnodejs rclnodejs-web web.json
```

> The `expose` block is the **public API** your browser depends on.
> Anything not listed is rejected with `code: 'not_exposed'` before
> any ROS 2 API runs. Keep it narrow.

The CLI does not launch ROS service/action servers or a background topic
publisher; run matching nodes separately, or use the self-contained demos. Set CORS to your
frontend's origin (here `http://localhost:8080`); CORS is not authorization.

## 2. Client side: talk to it from the browser

### Connect

```ts
import type {} from 'rclnodejs';
import { connect } from 'rclnodejs/web'; // or via esm.sh in a <script type="module">
```

The type-only import supplies ROS declarations; omit it in JavaScript.

Select transports with `connect()`:

| You want…                           | Pass                                                            |
| ----------------------------------- | --------------------------------------------------------------- |
| WebSocket only                      | `'ws://host:9000/capability'`                                   |
| HTTP + derived WS for subscriptions | `'http://host:9001'`                                            |
| HTTP + WS on different ports        | `{ http: 'http://host:9001', ws: 'ws://host:9000/capability' }` |
| HTTP only (no `subscribe()`)        | `{ http: 'http://host:9001' }`                                  |

A bare `http://` URL auto-derives the WS sibling at the same origin
(`/capability` path); the `{ http }`-only form disables WS entirely
and `subscribe()` rejects with `transport_unavailable`.

Actions use SSE with HTTP URLs or `{ http }`, and WS with an explicit
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
and reply from the generated ROS declarations (no additional frontend
codegen or hand-written shared types module). From plain JavaScript, drop the generic and
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

Start and expose the matching ROS action server first. For a complete app,
see the [Fibonacci browser demo](../demo/web/javascript/README.md#fibonacci-actions).

```ts
const httpClient = await connect({ http: 'http://localhost:9001' });
try {
  const goal = await httpClient.action<'example_interfaces/action/Fibonacci'>(
    '/fibonacci',
    { order: 5 },
    { onFeedback: (feedback) => console.log(feedback.sequence) }
  );
  const result = await goal.result;
  console.log(goal.status, result.sequence);
} finally {
  await httpClient.close();
}
```

HTTP actions use POST/SSE via `fetch()`, without `--http-sse`.
`EventSource` cannot POST goals. Feedback may precede `accepted`.
Request errors reject `action()`; stream errors reject `goal.result`.
Check `goal.status`: resolved results may be canceled or aborted.

For cancellation, use the `{ http, ws }` connection above:

```ts
const goal = await ros.action<'example_interfaces/action/Fibonacci'>(
  '/fibonacci',
  {
    order: 10,
  }
);
await goal.cancel();
console.log(await goal.result, goal.status);
```

The server may reject cancellation. HTTP `cancel()` rejects with `unsupported_kind`.

### Lifecycle and cleanup

`sub.close()` ends one subscription; `ros.close()` closes all subscriptions
and transports. Pending HTTP actions reject with `connection_lost`, but
ROS goals are not canceled.

Close clients during application/component teardown; browser unload cleanup
is best-effort.

```ts
const sub = await ros.subscribe('/chatter', handler);
// …
await sub.close(); // drop just this subscription
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

# Action (feedback and result over SSE)
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
the runtime. Type resolution still requires the sourced ROS environment and
available interfaces:

```bash
npx -p rclnodejs rclnodejs-web openapi web.json > openapi.json
```

SSE schemas describe individual event payloads; API explorers may buffer streams.

See [`demo/web/javascript/`](../demo/web/javascript/) for a full
walkthrough, including browsing it in Swagger UI.

## 4. `rclnodejs/web` vs. `rosbridge` + `roslibjs`

`rosbridge` + `roslibjs` is an established browser-side ROS stack.
Both stacks target the same job (talk to ROS 2 from a web
app over WebSocket + JSON) and both keep the browser facing
topics/services rather than inventing a higher-level abstraction. What
differs is **what's exposed to the browser, how strongly it's typed,
and whether plain HTTP works**:

|                             | **`rclnodejs/web`**                                                                            | `rosbridge` + `roslibjs`                                                                                      |
| --------------------------- | ---------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------- |
| **Public API surface**      | **`web.json` per-operation allow-list with explicit ROS types**                                | Graph-oriented protocol; access can be restricted by configured glob filters                                  |
| **TypeScript types**        | ROS type-name generics derive message, service and action payloads from generated declarations | Recent versions provide TypeScript types and payload generics; payload shapes are supplied by the application |
| **HTTP `call` / `publish`** | Built-in HTTP POST endpoints for exposed capabilities                                          | The standard WebSocket stack does not provide equivalent HTTP POST endpoints                                  |

See the upstream [rosbridge filtering configuration](https://github.com/RobotWebTools/rosbridge_suite/blob/HEAD/rosbridge_server/launch/rosbridge_websocket_launch.xml)
and [roslibjs typed service API](https://github.com/RobotWebTools/roslibjs/blob/HEAD/packages/roslib/src/core/Service.ts).
