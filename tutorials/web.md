# rclnodejs/web — Browser SDK guide

> Front-end-developer guide to talking to ROS 2 from a web app using
> `rclnodejs/web` — the typed Browser SDK and capability runtime
> shipped inside rclnodejs.
>
> This tutorial is _only_ about how to drive the runtime from the
> browser. For the SDK source see
> [`web/client.js`](../web/client.js); for runnable demos see
> [`demo/web/`](../demo/web/).

## What you get

- A 100-line ESM module (`rclnodejs/web`) with **zero native deps** —
  safe to bundle for any browser, works in any modern bundler (Vite,
  Next, esbuild, webpack), and importable from `<script type="module">`
  via a CDN.
- Three verbs: `call`, `publish`, `subscribe`.
- Two transports under the hood — **WebSocket** (long-lived, supports
  subscribe) and **HTTP** (stateless `call` / `publish`, curl-able).
  You pick which by the URL scheme; the SDK handles the rest.
- TypeScript types **derived from rclnodejs's auto-generated
  `MessagesMap` / `ServicesMap`** so a single string generic gives you
  request, response, and message shapes for free.

What you _do not_ write:

- A WebSocket / HTTP server — `rclnodejs-web` is the launcher.
- Glue code mapping ROS types to TS types — the generic does it.
- A reconnect loop (not yet implemented — see §8).

## 1. Server side, in five lines

You need a `rclnodejs-web` server running. From any project that has
`rclnodejs` installed:

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

Or with a `web.json` file:

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

Capabilities **not** in the allow-list are rejected with
`code: 'not_exposed'` before any ROS 2 API runs. That allow-list is
the public API surface your browser depends on — keep it narrow.

## 2. Install the SDK

```bash
npm install rclnodejs
```

Then in the browser:

```ts
import { connect } from 'rclnodejs/web';
```

Or without a bundler, via a CDN like esm.sh:

```html
<script type="module">
  import { connect } from 'https://esm.sh/rclnodejs/web';
</script>
```

> The SDK is `web/index.js` (ESM, `type: module`). It does **not**
> import the rclnodejs native binding, so it bundles cleanly.

## 3. Connect — pick the right URL form

`connect()` accepts three shapes. Choose based on which transport(s)
you actually want and where they live.

### A — WebSocket only (most common)

```ts
const ros = await connect('ws://localhost:9000/capability');
```

All three verbs go over a single WebSocket. Use this if `subscribe`
is the dominant use case (UI dashboards, telemetry, robot state).

### B — HTTP for `call`/`publish`, WebSocket lazily on first `subscribe`

```ts
const ros = await connect('http://localhost:9001');
```

- `call` and `publish` go over HTTP (stateless `fetch`).
- The first `subscribe()` lazily opens a WebSocket sibling at
  `ws://<same-host>:<same-port>/capability`.
- If you never `subscribe()`, no WebSocket is ever opened.

> ⚠️ Form B's auto-derived WS URL only works when HTTP and WS share
> a host:port (typical behind a reverse proxy). The default
> `rclnodejs-web` dev layout puts WS on `:9000` and HTTP on `:9001`
> — different ports — so `subscribe()` would fail with
> `code: 'transport_unavailable'`. Use Form C below for that layout.

### C — Split endpoints (the form to use with the default layout)

```ts
const ros = await connect({
  http: 'http://localhost:9001',
  ws: 'ws://localhost:9000/capability',
});
```

Spell out both URLs explicitly. This is the right form for the
default `--port` + `--http-port` deployment, or whenever HTTP and WS
sit behind different proxies / TLS terminations.

You can pass just `{ http }` (call/publish only — `subscribe` will
throw `code: 'transport_unavailable'`) or just `{ ws }` (same as
Form A).

### Decision table

| You want…                                                   | Use                       |
| ----------------------------------------------------------- | ------------------------- |
| WebSocket-only (the simplest setup)                         | Form A (`ws://...`)       |
| HTTP + WS behind one reverse proxy (shared host:port)       | Form B (`http://...`)     |
| `rclnodejs-web` default `--port` + `--http-port` dev layout | **Form C** (`{http, ws}`) |
| HTTP RPC only, no `subscribe()` ever                        | Form C with only `{http}` |

## 4. Call a service

```ts
const reply = await ros.call('/add_two_ints', { a: '7n', b: '35n' });
console.log(reply.sum); // '42n'
```

Typed (recommended):

```ts
const reply = await ros.call<'example_interfaces/srv/AddTwoInts'>(
  '/add_two_ints',
  { a: '7n', b: '35n' } // ← request typed as { a: `${number}n`, b: `${number}n` }
);
reply.sum; // ← typed as `${number}n`
```

The single string generic is the **ROS service type name**. The
SDK derives request and response shapes from rclnodejs's generated
`ServicesMap`, so you don't write or import any types yourself.

> 💡 **The `"42n"` convention.** ROS 2 64-bit integer fields don't
> survive JSON, so the runtime serialises them as `"<n>n"` strings on
> the wire (e.g. BigInt `42n` → string `"42n"`). The TypeScript type
> for these fields is `\`${number}n\``. JavaScript's `BigInt`literal
syntax happens to round-trip cleanly through`String(42n)`.

## 5. Publish to a topic

```ts
await ros.publish('/chatter', { data: 'hello from the browser' });
```

Resolves to `undefined` on success. On failure it throws an `Error`
with a `code` property (see §7), e.g. `code: 'not_exposed'` if the
topic isn't in the allow-list.

Typed:

```ts
await ros.publish<'std_msgs/msg/String'>('/chatter', { data: 'hello' });
// 2nd arg typed as { data: string }
```

## 6. Subscribe to a topic

```ts
const sub = await ros.subscribe('/chatter', (msg) => {
  console.log('recv:', msg.data);
});

// later, when you no longer need it:
await sub.close();
```

Typed:

```ts
const sub = await ros.subscribe<'std_msgs/msg/String'>(
  '/chatter',
  (msg) => console.log(msg.data) // msg: { data: string }
);
```

`sub` is `{ subId: string, close(): Promise<void> }`. The runtime
holds the underlying ROS 2 subscription open until `close()` (or
until the connection drops).

> Subscribing always uses WebSocket. If you connected over HTTP
> only and the WS sibling can't be reached, you'll get
> `code: 'transport_unavailable'` — see §3.

## 7. Errors

Every error thrown by the SDK has a stable `code` property in
addition to `message`. The codes you'll see most often:

| `code`                  | When                                                         |
| ----------------------- | ------------------------------------------------------------ |
| `not_exposed`           | Capability isn't in the server's `expose({...})` allow-list. |
| `not_implemented`       | Reserved kinds (e.g. `action`).                              |
| `unsupported_kind`      | You sent `subscribe` over HTTP — use WS.                     |
| `network_error`         | `fetch()` itself threw (DNS, refused, CORS).                 |
| `transport_unavailable` | `subscribe()` called with no WS sibling reachable.           |
| `invalid_response`      | Server replied non-JSON to a `call`.                         |
| `http_<status>`         | HTTP failure with no structured body (e.g. `http_502`).      |
| `call_failed`           | The ROS 2 service handler threw.                             |
| `publish_failed`        | The publisher rejected the message.                          |

```ts
try {
  await ros.call('/dangerous', {});
} catch (e) {
  const err = e as { code?: string; message: string };
  if (err.code === 'not_exposed') {
    // The capability isn't allow-listed — do not retry.
  } else {
    // Something else; surface to the user.
  }
}
```

The codes are stable across minor versions; treat any unknown code as
a non-retryable error and surface `e.message` to logs.

## 8. Connection lifecycle

```ts
const ros = await connect('ws://localhost:9000/capability');
// …use ros…
await ros.close(); // releases all subscriptions and closes the socket(s)
```

Today, **`reconnect: true` is accepted but ignored** (the SDK warns
to the console). If the server drops, you lose the client and have to
`connect()` again. Reconnect-on-close with automatic resubscribe is
on the roadmap.

```ts
const ros = await connect(endpoint, { reconnect: true });
// console: rclnodejs/web: reconnect is not yet implemented; ignoring option
```

## 9. End-to-end: a minimal app

```ts
import { connect } from 'rclnodejs/web';

const ros = await connect({
  http: 'http://localhost:9001', // call/publish over HTTP (curl-able)
  ws: 'ws://localhost:9000/capability', // subscribe over WS
});

const sumEl = document.querySelector<HTMLElement>('#sum')!;
const logEl = document.querySelector<HTMLElement>('#log')!;
const btnEl = document.querySelector<HTMLButtonElement>('#btn')!;

// Call a service.
const reply = await ros.call<'example_interfaces/srv/AddTwoInts'>(
  '/add_two_ints',
  { a: '2n', b: '40n' }
);
sumEl.textContent = reply.sum; // '42n'

// Subscribe to a topic.
const sub = await ros.subscribe<'std_msgs/msg/String'>('/chatter', (msg) => {
  const line = document.createElement('div');
  line.textContent = msg.data;
  logEl.appendChild(line);
});

// Publish on click.
btnEl.addEventListener('click', () =>
  ros.publish<'std_msgs/msg/String'>('/chatter', { data: 'hi' })
);

window.addEventListener('beforeunload', () => {
  sub.close();
  ros.close();
});
```

That's the full SDK surface.

## 10. When to use HTTP vs. WebSocket

| Job                                         | Transport        | Why                                                              |
| ------------------------------------------- | ---------------- | ---------------------------------------------------------------- |
| Service calls from a UI button              | either           | HTTP is curl-able for debugging; WS is one fewer connection.     |
| Publishing user input on click              | either           | Same.                                                            |
| Subscribing to telemetry / robot state      | WebSocket        | Server push; HTTP `subscribe` isn't supported by design.         |
| AI-agent or external tool driving the robot | HTTP             | No JS stack needed; just `curl -X POST /capability/call/<name>`. |
| Mixed (curl-able RPC + UI subscribe)        | both, via Form C | The SDK juggles them transparently.                              |

There is intentionally **no SSE transport** for subscribe-over-HTTP.
WebSocket already multiplexes N subscriptions on one socket and
carries binary frames — SSE would force one HTTP connection per
topic and would still not help with duplex flows like Actions.

## 11. curl recipes (no JavaScript at all)

Any HTTP client can drive the runtime as long as `--http-port` is on:

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

# Allow-list rejection (returns 404 + structured error body)
curl -sS -X POST http://localhost:9001/capability/call/dangerous \
  -H 'content-type: application/json' -d '{}'
# => {"ok":false,"error":"capability not exposed: call /dangerous","code":"not_exposed"}
```

## 12. Relationship to `rosocket`

`rclnodejs` ships **two** browser ↔ ROS 2 bridges. They are
**siblings**, not layers — independent tools that share the same
rclnodejs core but solve different problems. Run them as separate
processes on separate ports; they never talk to each other.

|                 | `rosocket`                              | `rclnodejs/web`                                                      |
| --------------- | --------------------------------------- | -------------------------------------------------------------------- |
| **What it is**  | A thin WebSocket bridge                 | A typed SDK + capability runtime                                     |
| **Browser API** | Hand-rolled `WebSocket + JSON`          | `import { connect } from 'rclnodejs/web'`                            |
| **URL shape**   | `/topic/<name>`, `/service/<name>`      | `/capability` (WS) + `POST /capability/{call,publish}/<name>` (HTTP) |
| **Allow-list**  | Optional type hints                     | **Required** — `web.json` is the contract                            |
| **Best for**    | Quick prototypes, `roslibjs`-style apps | New apps, AI agents, typed UIs                                       |

If you only need a couple of topics wired into a hand-rolled
`WebSocket` and don't want to pull in a TypeScript SDK, reach for
[`rosocket`](../rosocket/README.md). For new apps, AI-agent tool-use,
or anywhere you want a reviewable allow-list and end-to-end types,
use `rclnodejs/web`.

## 13. Relationship to `rosbridge` + `roslibjs`

`rclnodejs/web` and `rosbridge` + `roslibjs` ride the **same wire
transport** (WebSocket + JSON) but make a different contract with
the browser:

|                             | `rosbridge` + `roslibjs`                 | `rclnodejs/web`                                                                                  |
| --------------------------- | ---------------------------------------- | ------------------------------------------------------------------------------------------------ |
| **Public API surface**      | The whole ROS graph                      | `web.json` allow-list — a reviewable artifact                                                    |
| **TypeScript types**        | `any`-shaped; bolt-on community packages | Single string generic derives request / response / message from rclnodejs's auto-generated types |
| **HTTP `call` / `publish`** | ❌ no HTTP surface                       | ✅ `curl`, Postman, AI-agent tool-use just work                                                  |

**This is not a rosbridge replacement.** rosbridge is still the
right choice when you genuinely need graph-shaped semantics —
rqt-web, fleet dashboards that introspect topics, anything that has
to enumerate the live graph rather than work against a fixed
contract.

## See also

- [`web/client.js`](../web/client.js) — the SDK source (compact;
  worth a skim).
- [`demo/web/javascript/`](../demo/web/javascript/)
  — runnable demo, no build tools, with a transport toggle and curl
  recipes.
- [`demo/web/typescript/`](../demo/web/typescript/)
  — same demo with TypeScript + Vite, end-to-end typed.
- [`bin/rclnodejs-web.js`](../bin/rclnodejs-web.js) — the
  `rclnodejs-web` CLI; `rclnodejs-web --help` lists every flag and
  config-file key.
