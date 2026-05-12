# rclnodejs/web — Browser SDK guide

> Talk to ROS 2 from a web app — typed, allow-listed, no `roslibjs`.

`rclnodejs/web` is the browser-side of `rclnodejs`: a compact ESM
module (`web/client.js`) plus a server runtime (`bin/rclnodejs-web.js`)
that exposes a declarative subset of your ROS 2 graph over WebSocket
**and** plain HTTP. The browser API is three verbs — `call`,
`publish`, `subscribe` — typed end-to-end from rclnodejs's
auto-generated `MessagesMap` / `ServicesMap`.

For runnable code see [`demo/web/`](../demo/web/):

| Demo                                              | Pick this if you…                                                           |
| ------------------------------------------------- | --------------------------------------------------------------------------- |
| [`demo/web/javascript/`](../demo/web/javascript/) | want a single static page — no build tools, no `npm install` for the page   |
| [`demo/web/typescript/`](../demo/web/typescript/) | already have a Vite / Next / React / Vue / Svelte project, want full typing |

## 1. Stand up a server

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

## 2. Connect

```ts
import { connect } from 'rclnodejs/web'; // or via esm.sh in a <script type="module">
```

`connect()` accepts three URL shapes — the SDK picks transport(s)
from the scheme:

| You want…                                                     | Pass                                                                                     |
| ------------------------------------------------------------- | ---------------------------------------------------------------------------------------- |
| WebSocket only (the simplest setup)                           | `'ws://host:9000/capability'`                                                            |
| HTTP + WS behind one reverse proxy (shared host:port)         | `'http://host:9001'` — WS sibling derived as `ws://host:9001/<existing-path>/capability` |
| Default `--port` + `--http-port` dev layout (different ports) | `{ http: 'http://host:9001', ws: 'ws://host:9000/capability' }`                          |
| HTTP RPC only (no `subscribe()` ever)                         | `{ http: 'http://host:9001' }` — `subscribe()` rejects with `transport_unavailable`      |

```ts
const ros = await connect({
  http: 'http://localhost:9001',
  ws: 'ws://localhost:9000/capability',
});
```

## 3. The verb API

All three verbs accept an optional ROS-type generic that drives
typing of the payload and reply.

```ts
// Service call
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

The generic is omittable; without it, payload and reply are typed as
`unknown`. With it, every shape comes from rclnodejs's generated
maps — no codegen, no shared types module.

> 💡 **The `"42n"` convention.** ROS 2 64-bit integer fields don't
> survive JSON, so the runtime serialises them as the string template
> `` `${big}n` `` on the wire (BigInt `42n` → string `"42n"`). The
> matching TypeScript field type is `` `${number}n` ``.

## 4. Errors

Every thrown error has a stable `code`. The most common:

| `code`                  | When                                                    |
| ----------------------- | ------------------------------------------------------- |
| `not_exposed`           | Capability isn't in the server's allow-list.            |
| `transport_unavailable` | `subscribe()` called with no WS sibling reachable.      |
| `network_error`         | `fetch()` itself threw (DNS, refused, CORS).            |
| `invalid_response`      | Server replied non-JSON to a `call`.                    |
| `http_<status>`         | HTTP failure with no structured body (e.g. `http_502`). |
| `call_failed`           | The ROS 2 service handler threw.                        |
| `publish_failed`        | The publisher rejected the message.                     |

The runtime defines a few more (`unsupported_kind`, `invalid_frame`,
`payload_too_large`, …); treat any unknown code as non-retryable and
surface `e.message` to logs.

```ts
try {
  await ros.call('/dangerous', {});
} catch (e) {
  if ((e as { code?: string }).code === 'not_exposed') {
    // Allow-listed differently — don't retry.
  }
}
```

## 5. Connection lifecycle

```ts
await ros.close(); // cancels subscriptions, closes both transports
```

Today **`reconnect: true` is accepted but ignored** (the SDK warns
once). If the server drops, the client is dead — `connect()` again
and re-`subscribe()`. Auto-reconnect with resubscribe is on the
roadmap.

## 6. End-to-end

```ts
import { connect } from 'rclnodejs/web';

const ros = await connect({
  http: 'http://localhost:9001', // call/publish over HTTP
  ws: 'ws://localhost:9000/capability', // subscribe over WS
});

const reply = await ros.call<'example_interfaces/srv/AddTwoInts'>(
  '/add_two_ints',
  { a: '2n', b: '40n' }
);
document.querySelector('#sum')!.textContent = reply.sum; // '42n'

const sub = await ros.subscribe<'std_msgs/msg/String'>('/chatter', (msg) =>
  log(msg.data)
);

document
  .querySelector('#btn')!
  .addEventListener('click', () =>
    ros.publish<'std_msgs/msg/String'>('/chatter', { data: 'hi' })
  );

window.addEventListener('beforeunload', () => {
  sub.close();
  ros.close();
});
```

## 7. curl recipes (no JavaScript at all)

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

# Allow-list rejection (returns 404 + structured body)
curl -sS -X POST http://localhost:9001/capability/call/dangerous \
  -H 'content-type: application/json' -d '{}'
# => {"ok":false,"error":"capability not exposed: call /dangerous","code":"not_exposed"}
```

## 8. How it compares

`rclnodejs` ships **two** browser ↔ ROS 2 bridges, side-by-side
with the upstream `rosbridge` + `roslibjs` stack:

|                             | `rosocket`                              | **`rclnodejs/web`**                                                  | `rosbridge` + `roslibjs`                 |
| --------------------------- | --------------------------------------- | -------------------------------------------------------------------- | ---------------------------------------- |
| **Browser API**             | Hand-rolled `WebSocket + JSON`          | `import { connect } from 'rclnodejs/web'`                            | `roslibjs`                               |
| **URL shape**               | `/topic/<name>`, `/service/<name>`      | `/capability` (WS) + `POST /capability/{call,publish}/<name>` (HTTP) | rosbridge protocol over WS               |
| **Public surface**          | All listed topics/services              | **Required `web.json` allow-list** — a reviewable artifact           | The whole ROS graph                      |
| **TypeScript types**        | `any`-shaped                            | Single string generic → request/response/message                     | `any`-shaped; bolt-on community packages |
| **HTTP `call` / `publish`** | ❌                                      | ✅ — `curl`, Postman, AI-agent tool-use just work                    | ❌                                       |
| **Best for**                | Quick prototypes, `roslibjs`-style apps | New apps, AI agents, typed UIs                                       | Graph introspection, rqt-web, fleet UIs  |

`rosocket` and `rclnodejs/web` are **siblings**, not layers — run as
separate processes on separate ports. `rosbridge` remains the right
choice when you genuinely need graph-shaped semantics (anything that
has to enumerate the live graph rather than work against a fixed
contract).

## 9. Auth, HTTPS, Actions

- **HTTPS / `wss://`.** The runtime speaks plain `ws://` and
  `http://`. Put nginx, Caddy, or any TLS proxy in front of
  `rclnodejs-web`; clients then point at `wss://` / `https://`.
- **Auth.** Today, gate at the connection level via the
  `verifyClient(req)` / `verifyRequest(req)` hooks on
  `WebSocketTransport` / `HttpTransport` (return `false` to reject
  with a 401). Per-capability scopes are on the roadmap.
- **Browser ROS install?** No — the browser only ever speaks to the
  endpoint `rclnodejs-web` exposes.
- **Actions.** Not yet — reserved as the `action` kind in the wire
  protocol; coming in a follow-up release.

## See also

- [`web/client.js`](../web/client.js) — the SDK source, worth a skim.
- [`demo/web/javascript/`](../demo/web/javascript/) — runnable demo,
  no build tools, transport toggle + curl recipes.
- [`demo/web/typescript/`](../demo/web/typescript/) — same demo with
  TypeScript + Vite, end-to-end typed.
- [`bin/rclnodejs-web.js`](../bin/rclnodejs-web.js) — the
  `rclnodejs-web` CLI; `rclnodejs-web --help` lists every flag and
  every config-file key.
