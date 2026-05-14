# rclnodejs/web — Browser SDK guide

> Talk to ROS 2 from a web app — typed, allow-listed, `curl`-able.

`rclnodejs/web` is the browser-side of `rclnodejs`: a compact ESM
module plus a server runtime that together expose a declarative
subset of your ROS 2 graph over WebSocket **and** plain HTTP. The
browser API is three verbs — `call`, `publish`, `subscribe` — typed
end-to-end from your ROS 2 message and service types.

For runnable code see [`demo/web/`](../demo/web/):

| Demo                                              | Pick this if you…                                                           |
| ------------------------------------------------- | --------------------------------------------------------------------------- |
| [`demo/web/javascript/`](../demo/web/javascript/) | want a single static page — no build tools, no `npm install` for the page   |
| [`demo/web/typescript/`](../demo/web/typescript/) | already have a Vite / Next / React / Vue / Svelte project, want full typing |

## 1. Server side: stand up the runtime

```bash
source /opt/ros/<distro>/setup.bash
npx rclnodejs-web \
  --port 9000 --http-port 9001 \
  --call /add_two_ints=example_interfaces/srv/AddTwoInts \
  --publish /chatter=std_msgs/msg/String \
  --subscribe /scan=sensor_msgs/msg/LaserScan
# rclnodejs/web listening on ws://localhost:9000/capability (3 capabilities)
#                also http://localhost:9001/capability (call/publish only)
```

Or feed the same allow-list from `web.json`:

```json
{
  "port": 9000,
  "http": { "port": 9001 },
  "expose": {
    "call": { "/add_two_ints": "example_interfaces/srv/AddTwoInts" },
    "publish": { "/chatter": "std_msgs/msg/String" },
    "subscribe": { "/scan": "sensor_msgs/msg/LaserScan" }
  }
}
```

```bash
npx rclnodejs-web web.json
```

> The `expose` block is the **public API** your browser depends on.
> Anything not listed is rejected with `code: 'not_exposed'` before
> any ROS 2 API runs. Keep it narrow.

## 2. Client side: talk to it from the browser

### Connect

```ts
import { connect } from 'rclnodejs/web'; // or via esm.sh in a <script type="module">
```

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

### Lifecycle and cleanup

Each `subscribe()` returns a handle with its own `close()`; the
top-level `ros.close()` cancels every active subscription and shuts
down both transports.

```ts
const sub = await ros.subscribe('/chatter', handler);
// …
sub.close(); // drop just this subscription
await ros.close(); // tear down the whole connection

// Typical browser cleanup:
window.addEventListener('beforeunload', () => ros.close());
```

## 3. curl recipes (no JavaScript at all)

When `--http-port` is on, every `call` / `publish` is reachable from
any HTTP client — curl, Postman, AI-agent tool-use, no SDK required.
Subscribe stays on WebSocket.

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
```

## 4. How it compares

The closest neighbour in this space is `rosbridge` + `roslibjs` — the
standard browser-side ROS 2 stack for the past decade. Both target the
same job (talk to ROS 2 from a web app over WebSocket + JSON) and
both deliberately keep the browser facing topics/services rather
than inventing a higher-level abstraction. What differs is **what's
exposed to the browser, how strongly it's typed, and whether plain
HTTP works**:

|                             | **`rclnodejs/web`**                                                  | `rosbridge` + `roslibjs`          |
| --------------------------- | -------------------------------------------------------------------- | --------------------------------- |
| **Public API surface**      | **`web.json` allow-list — reviewable artifact**                      | The whole live ROS graph          |
| **TypeScript types**        | One ROS 2 type name → fully typed request/response/message | `any`; bolt-on community packages |
| **HTTP `call` / `publish`** | ✅ — `curl`, Postman, AI-agent tool-use just work                    | ❌ (WebSocket only)               |
