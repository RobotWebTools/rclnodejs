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
and re-`subscribe()`.

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

The browser ↔ ROS 2 space already has `rosbridge` + `roslibjs`, and
`rclnodejs` itself ships a second, lighter bridge called `rosocket`.
All three speak to the same ROS graph — the differences live in **the
contract you sign with the browser**, not in transport tricks.

> 💡 **Why we don't try to hide the ROS graph.** Robotics has no
> stable equivalent to the web's URL/DOM/REST/SQL primitives, so any
> attempt at a fully product-agnostic `robot.navigate(...)` style API
> ends in unbounded capability explosion. All three options below
> therefore keep the browser facing topics/services/actions
> deliberately. The differentiator is the operational surface
> _around_ the graph (allow-list, types, transports, governance), not
> a new frontend abstraction. `robot.navigate()`-style verbs belong
> in the integrator's product layer on top of these.

### Pick by the contract you want

| You want…                                                                       | Use                                                                   |
| ------------------------------------------------------------------------------- | --------------------------------------------------------------------- |
| **A reviewable allow-list, typed SDK, and curl-able HTTP** for new browser apps | **`rclnodejs/web`** _(this guide; default for new code)_              |
| Live graph introspection / debug consoles / `rqt`-web / fleet topic discovery   | `rosbridge` + `roslibjs`                                              |
| Hand-rolled `WebSocket + JSON` against a couple of named topics, no SDK at all  | [`rosocket`](../rosocket/)                                            |
| You already have a working `roslibjs` codebase with no migration pressure       | Stay on `rosbridge` + `roslibjs`                                      |
| You need ROS 2 Actions                                                          | `rosbridge` + `roslibjs` _(rclnodejs/web does not implement actions)_ |

### Detailed matrix

|                             | **`rclnodejs/web`** _(default)_                                      | [`rosocket`](../rosocket/)              | `rosbridge` + `roslibjs`                                         |
| --------------------------- | -------------------------------------------------------------------- | --------------------------------------- | ---------------------------------------------------------------- |
| **Public API surface**      | **`web.json` allow-list — reviewable artifact**                      | All listed topics/services              | The whole live ROS graph                                         |
| **Browser API**             | `import { connect } from 'rclnodejs/web'` — typed, three verbs       | Hand-rolled `WebSocket + JSON`          | `roslibjs`                                                       |
| **TypeScript types**        | Single string generic → request/response/message from generated maps | `any`-shaped                            | `any`; bolt-on community packages                                |
| **Wire shape**              | `/capability` (WS) + `POST /capability/{call,publish}/<name>` (HTTP) | `/topic/<name>`, `/service/<name>` (WS) | rosbridge protocol over WS                                       |
| **HTTP `call` / `publish`** | ✅ — `curl`, Postman, AI-agent tool-use just work                    | ❌                                      | ❌                                                               |
| **Setup**                   | `npx rclnodejs-web web.json`                                         | `npx rosocket --topic …`                | `apt install ros-<distro>-rosbridge-server` + Python launch file |
| **Auth hooks**              | `verifyClient` / `verifyRequest` per transport (today)               | Connection-level                        | `securityglobs` plugin                                           |
| **Mature / battle-tested**  | New (2.0)                                                            | New (2.0)                               | 10+ years in production                                          |

### Sibling, not competitor: `rosocket`

`rosocket` and `rclnodejs/web` ship in the **same** rclnodejs package
but are independent runtimes — different ports, different contracts,
no shared state. Reach for `rosocket` when you genuinely just want
"one named topic over a raw WebSocket"; reach for `rclnodejs/web`
when you want a typed SDK and a reviewable allow-list. Neither
replaces the other.

> **The URL shapes are deliberately different**, not a missed
> opportunity to unify. `rosocket`'s `ws://host:9000/topic/<name>`
> means "one socket per resource, the path **is** the resource";
> `rclnodejs/web`'s `ws://host:9000/capability` means "one socket per
> session, the resource lives in the message frame so call/publish/
> subscribe can multiplex." Same wire (WebSocket + JSON), different
> multiplexing model. The URL is the user-visible signal of which
> contract you're talking to.

### Not a `rosbridge` replacement

`rosbridge` is still the right tool when the browser genuinely needs
**graph-shaped semantics** — enumerating live topics, building
`rqt`-style debug UIs, fleet dashboards that have to discover what's
running. `rclnodejs/web` deliberately gives that up in exchange for a
narrow, declarative contract.

## 9. Auth, HTTPS, Actions

- **HTTPS / `wss://`.** The runtime speaks plain `ws://` and
  `http://`. Put nginx, Caddy, or any TLS proxy in front of
  `rclnodejs-web`; clients then point at `wss://` / `https://`.
- **Auth.** Gate at the connection level via the
  `verifyClient(req)` / `verifyRequest(req)` hooks on
  `WebSocketTransport` / `HttpTransport` (return `false` to reject
  with a 401).
- **Browser ROS install?** No — the browser only ever speaks to the
  endpoint `rclnodejs-web` exposes.
- **Actions.** Not implemented. Use `rosbridge` + `roslibjs` if you
  need ROS 2 Actions in the browser today.

## See also

- [`web/client.js`](../web/client.js) — the SDK source, worth a skim.
- [`demo/web/javascript/`](../demo/web/javascript/) — runnable demo,
  no build tools, transport toggle + curl recipes.
- [`demo/web/typescript/`](../demo/web/typescript/) — same demo with
  TypeScript + Vite, end-to-end typed.
- [`bin/rclnodejs-web.js`](../bin/rclnodejs-web.js) — the
  `rclnodejs-web` CLI; `rclnodejs-web --help` lists every flag and
  every config-file key.
