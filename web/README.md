# rclnodejs/web

A browser client for ROS 2 services, topics, and actions. The Node.js runtime, `rclnodejs/web/server`, exposes configured operations over WebSocket and HTTP. ROS declarations supply TypeScript types. Neither these types nor OpenAPI schemas validate payloads at runtime.

Examples in [`demo/web/`](../demo/web/):

| Demo | Setup |
| --- | --- |
| [API playground (JavaScript)](../demo/web/api-playground/javascript/) | Static page; no frontend build |
| [API playground (TypeScript)](../demo/web/api-playground/typescript/) | Vite app with generated ROS types |
| [Manipulator](../demo/web/manipulator/) | Terminal keyboard control and a read-only Three.js view |

## Transport Support

The table covers web-client roles, not the full native API. HTTP paths use the default `/capability` base path.

| ROS 2 role / operation                              | WebSocket endpoint                          | HTTP / SSE endpoint                                                                         | SDK usage and limitations                                                                |
| --------------------------------------------------- | ------------------------------------------- | ------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------- |
| Pub/Sub: topic publisher                            | Supported                                   | HTTP `POST /capability/publish/<name>`; 204 on success                                      | `ros.publish(name, message)`                                                             |
| Pub/Sub: topic subscriber                           | Supported; topics share one connection      | HTTP `GET /capability/subscribe/<name>` with SSE; requires `--http-sse` or `http.sse: true` | `ros.subscribe()` uses WS only; HTTP consumers use `EventSource` or curl.            |
| Client/Service: service client                      | Supported                                   | HTTP `POST /capability/call/<name>`; JSON response                                          | `ros.call(name, request)` calls native ROS service servers.                              |
| Client/Service: browser-hosted service server       | Not exposed                                 | Not exposed                                                                                 | Browser request handling and response/disconnect lifecycle are not implemented.          |
| Action client: send goals, receive feedback/results | Supported                                   | HTTP `POST /capability/action/<name>` with SSE; no `--http-sse` required                | `ros.action(name, goal, { onFeedback })`; await `goal.result`, then check `goal.status`. |
| Action: client cancellation                         | Supported, subject to ROS server acceptance | Not supported                                                                           | `goal.cancel()` requires WS; HTTP rejects with `unsupported_kind`.                       |
| Action: browser-hosted action server                | Not exposed                                 | Not exposed                                                                                 | Browser goal execution and feedback/result/cancel lifecycle are not implemented.         |

Service and action servers run on the ROS side.

## Server

Use `npx -p rclnodejs rclnodejs-web`, or `npx rclnodejs-web` when the package is installed locally. From a prepared source checkout, use `node bin/rclnodejs-web.js` instead.

```bash
source /opt/ros/<distro>/setup.bash
npx -p rclnodejs rclnodejs-web \
  --port 9000 --http-port 9001 \
  --http-sse --http-cors http://localhost:8080 \
  --call /add_two_ints=example_interfaces/srv/AddTwoInts \
  --publish /chatter=std_msgs/msg/String \
  --subscribe /chatter=std_msgs/msg/String \
  --action /fibonacci=example_interfaces/action/Fibonacci
```

The same configuration in `web.json`:

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

`expose` is an allow-list. Unlisted operations fail with `code: 'not_exposed'` before dispatch to ROS 2.

Run the matching ROS service/action servers and topic publishers separately, or use the self-contained demos. Set CORS to your frontend's origin (here `http://localhost:8080`); CORS is not authorization.

## Client

### Connect

```ts
import type {} from 'rclnodejs';
import { connect } from 'rclnodejs/web';
```

The type-only import supplies ROS declarations; omit it in JavaScript.

| Transport                           | Argument                                                        |
| ----------------------------------- | --------------------------------------------------------------- |
| WebSocket only                      | `'ws://host:9000/capability'`                                   |
| HTTP + derived WS for subscriptions | `'http://host:9001'`                                            |
| HTTP + WS on different ports        | `{ http: 'http://host:9001', ws: 'ws://host:9000/capability' }` |
| HTTP only (no `subscribe()`)        | `{ http: 'http://host:9001' }`                                  |

A bare HTTP URL derives a WebSocket URL at `/capability` on the same origin. Use `{ http }` to disable WebSocket; `subscribe()` then rejects with `transport_unavailable`.

Actions use SSE with HTTP URLs or `{ http }`, and WS with an explicit `{ http, ws }` pair.

```ts
const ros = await connect({
  http: 'http://localhost:9001',
  ws: 'ws://localhost:9000/capability',
});
```

### Operations

ROS type-name generics select payload and reply types from the generated declarations. In JavaScript, omit the generics. Encode 64-bit integers as strings such as `'7n'` for JSON transport.

```ts
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

Run and expose a ROS action server first. See the [Fibonacci demo](../demo/web/api-playground/javascript/README.md#fibonacci-actions).

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

HTTP actions use POST/SSE via `fetch()`; `--http-sse` is not needed. `EventSource` cannot POST goals. Feedback may arrive before `accepted`. Request errors reject `action()`; stream errors reject `goal.result`. Check `goal.status` even when the result resolves: it may be canceled or aborted. Disconnecting does not cancel the ROS goal.

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

`sub.close()` ends one subscription; `ros.close()` closes all subscriptions and transports. Pending HTTP actions reject with `connection_lost` without canceling their ROS goals.

Close clients during application or component teardown. Browser unload cleanup is best-effort.

## HTTP Examples

`--http-port` enables service calls, publishing, and SSE actions. Add `--http-sse` (or `"http": { "sse": true }`) for SSE topic subscriptions.

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

Use `EventSource` for browser subscriptions. Set `--http-cors` for cross-origin requests:

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

Export the `expose` configuration as OpenAPI 3.1 without starting the runtime. Source a ROS environment containing the referenced interfaces:

```bash
npx -p rclnodejs rclnodejs-web openapi web.json > openapi.json
```

SSE schemas describe individual event payloads; API explorers may buffer streams.

The [JavaScript API playground](../demo/web/api-playground/javascript/) includes a Swagger UI walkthrough.

## Comparison with rosbridge

|                             | `rclnodejs/web`                                                                            | `rosbridge` + `roslibjs`                                                                                      |
| --------------------------- | ---------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------- |
| API surface      | `web.json` per-operation allow-list with explicit ROS types                                | Graph-oriented protocol; access can be restricted by configured glob filters                                  |
| TypeScript types        | ROS type-name generics derive message, service and action payloads from generated declarations | Recent versions provide TypeScript types and payload generics; payload shapes are supplied by the application |
| HTTP `call` / `publish` | Built-in HTTP POST endpoints for exposed capabilities                                          | The standard WebSocket stack does not provide equivalent HTTP POST endpoints                                  |

See the upstream [rosbridge filtering configuration](https://github.com/RobotWebTools/rosbridge_suite/blob/HEAD/rosbridge_server/launch/rosbridge_websocket_launch.xml) and [roslibjs typed service API](https://github.com/RobotWebTools/roslibjs/blob/HEAD/packages/roslib/src/core/Service.ts).
