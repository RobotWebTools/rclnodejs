# Web Runtime Roadmap

> Implementation plan for the Web Runtime layer of rclnodejs, mapped to
> concrete releases. Companion to [WEB_RUNTIME_ARCH.md](./WEB_RUNTIME_ARCH.md)
> (architecture rationale) and
> [RCLNODEJS_WEB_VS_ROSBRIDGE.md](./RCLNODEJS_WEB_VS_ROSBRIDGE.md)
> (differentiation vs. existing tools).

## What we are claiming, and what we aren't

A reader can fairly look at 2.0.0 and say: _"You have a Node process
exposing a WebSocket that browsers connect to via a JS client. That's
rosbridge + roslibjs."_ The topology is the same — the
**differentiation is narrower than the eventual "web-native runtime"
pitch implies, and we should not overclaim it before we ship it.**

| Claim                                                                | Honest in 2.0.0? | Why                                                                                       |
| -------------------------------------------------------------------- | :--------------: | ----------------------------------------------------------------------------------------- |
| Capability allow-list as the API contract (`runtime.expose({...})`)  |        ✅        | Real today; no rosbridge-style "expose the ROS graph wholesale".                          |
| Typed Browser SDK derived from generated `MessagesMap`/`ServicesMap` |        ✅        | Real today; roslibjs has no equivalent.                                                   |
| Zero-server-code launch via `rclnodejs-web` CLI + `web.json`         |        ✅        | Real today; no Python `rosbridge_server` install needed.                                  |
| Transport-agnostic capability runtime (HTTP + WS)                    |        ✅        | Both transports ship in 2.0.0 — WS for `subscribe`, HTTP **and** WS for `call`/`publish`. |
| OpenAPI / introspectable Web API                                     |  ❌ until 2.2.0  | Registry has the right shape; emitter not yet built.                                      |
| AI-agent friendly (MCP, schema-validated calls)                      | ❌ until 2.2.0+  | Same.                                                                                     |

**2.0.0 framing**: `rclnodejs/web` is a _typed Web SDK +
transport-agnostic capability runtime_ built into rclnodejs. It
speaks **WebSocket and HTTP** today (HTTP for `call`/`publish`, WS
for everything including `subscribe`). The contract between server
and browser is a declarative allow-list (`web.json`), not an open
protocol bridge.

**Things we will not say in 2.0.0 announcements**: "OpenAPI-described
Web API" (true after 2.2.0, not 2.0.0), "rosbridge replacement"
(invites a feature-parity argument we don't want), "ROS 2 BFF layer"
(too abstract; nobody has checked our work yet).

## Snapshot

| Version        | Status              | Theme                                                                                                |
| -------------- | ------------------- | ---------------------------------------------------------------------------------------------------- |
| `2.0.0-beta.0` | shipped (May 2025)  | rosocket transport (Layer 2)                                                                         |
| `2.0.0`        | shipping (May 2026) | Capability runtime (L3) + Browser SDK (L4) + WebSocket **and** HTTP transports + `rclnodejs-web` CLI |
| `2.1.0`        | planned             | Production-readiness: Actions, reconnect, observability                                              |
| `2.2.0`        | planned             | Schema validation, OpenAPI export, name→type inference                                               |
| `2.3.0+`       | planned             | IAM scopes, MCP server, CBOR, QoS surfacing                                                          |

## 2.0.0-beta.0 — rosocket (shipped)

**Layer 2 only.** Resource-style WebSocket gateway: `ws://host:port/topic/<name>`,
`ws://host:port/service/<name>`. JSON payloads with the BigInt `"Nn"`
convention. `verifyClient(req)` hook for connection-level auth.

- [rosocket/index.js](../rosocket/index.js) — server.
- [rosocket/cli.js](../rosocket/cli.js) — `npx rosocket`.
- [rosocket/README.md](../rosocket/README.md) — URL scheme + service-call
  envelope.

**Limitations carried forward**: no allow-list, no schema enforcement,
no client SDK.

## 2.0.0 — Capability runtime + Browser SDK

**Layers 3 + 4 over the WebSocket and HTTP transports.** First vertical
slice through all four architectural layers; the seam at Layer 2 is
proven by shipping two adapters at once.

### Already implemented on `develop`

- **L3 — Capability registry**: `runtime.expose({ call, publish, subscribe })`
  declarative allow-list. Capabilities not in the registry are rejected
  with `code: 'not_exposed'` before any `Node.create*()` call runs.
  ([lib/runtime/registry.js](../lib/runtime/registry.js))
- **L3 — Dispatcher**: per-connection state, lazy creation/teardown of
  publishers/subscriptions/clients, BigInt rehydration, reserved
  `kind: 'action'` returning `code: 'not_implemented'` so 2.1.x can land
  Actions without a wire-protocol bump.
  ([lib/runtime/dispatcher.js](../lib/runtime/dispatcher.js))
- **L2 seam — `TransportAdapter` + `Connection` base classes**: the only
  contract Layer 3 sees. Both the WebSocket and HTTP transports
  implement this interface, so Layer 3 is transport-agnostic.
  ([lib/runtime/transport.js](../lib/runtime/transport.js))
- **L2 — `WebSocketTransport`**: ws-backed adapter on `/capability`.
  ([lib/runtime/transports/ws.js](../lib/runtime/transports/ws.js))
- **L2 — `HttpTransport`**: stateless HTTP adapter for `call`/`publish`.
  `POST /capability/call/<name>` returns the JSON reply (200) or a
  structured `{error, code}` body with the appropriate status
  (404 `not_exposed`, 501 `not_implemented`, etc.). `POST
/capability/publish/<name>` returns 204 on success. Subscriptions
  intentionally stay on `WebSocketTransport` (see "Why no SSE?").
  Same `Dispatcher` / `CapabilityRegistry` underneath — the L2 seam
  is now proven with two adapters.
  ([lib/runtime/transports/http.js](../lib/runtime/transports/http.js))
- **L4 — Browser SDK**: pure-JS client (`globalThis.WebSocket` in
  browsers, `ws` fallback in Node; `fetch` for HTTP). Verb API
  (`call` / `publish` / `subscribe`), object-handle API
  (`ros.topic(name)` / `ros.service(name)` in roslibjs style),
  UUID frame ids, `reconnect:false` default with warning if user
  opts in. **Transport is picked from the URL scheme**: `ws://` /
  `wss://` for WebSocket-only, `http://` / `https://` for HTTP
  `call`/`publish` (subscribe transparently uses a sibling WS
  endpoint), or `{http, ws}` to specify both explicitly.
  ([web/client.js](../web/client.js))
- **Tests**: 46 in the web suite (25 CLI + 12 HTTP + 9 WS), 53 with
  rosocket. Unit coverage on the registry; end-to-end service-call,
  pub/sub, capability rejection, distinct-id, action reservation,
  object-handle, raw HTTP wire format, SDK-over-HTTP, allow-list
  rejection over HTTP, mixed `{http, ws}` pairing, CLI argv parsing,
  config validation, end-to-end CLI launch with both transports.
  ([test/test-web.js](../test/test-web.js),
  [test/test-web-http.js](../test/test-web-http.js),
  [test/test-web-cli.js](../test/test-web-cli.js))
- **Wire protocol locked**:
  `kind ∈ {'call','publish','subscribe','unsubscribe','action'}`,
  ids opaque strings, errors as `{ok:false, error, code}` with stable
  codes (`not_exposed`, `not_implemented`, `unknown_kind`,
  `invalid_frame`, `invalid_json`, `binary_unsupported`, `missing_id`,
  `duplicate_id`, `unknown_sub_id`, `missing_sub_id`, `internal_error`,
  `call_failed`, `publish_failed`).
  The HTTP transport adds the following codes (additive; same
  `{ok:false, error, code}` body shape, status code as listed):
  `unsupported_kind` (404 — `subscribe`/`action` over HTTP),
  `not_found` (404 — outside `basePath`),
  `method_not_allowed` (405),
  `invalid_content_type` (400),
  `invalid_url` (400),
  `payload_too_large` (413),
  `streaming_unsupported` (400 — defensive),
  `unauthorized` (401 — from `verifyRequest` hook).
  The Browser SDK additionally surfaces `network_error` (fetch threw),
  `invalid_response` (server replied non-JSON to a `call`),
  `transport_unavailable` (subscribe attempted with no WS sibling),
  and `http_<status>` (un-coded HTTP failure).
- **Subpath exports** in `package.json`:
  `rclnodejs/web` (browser SDK), `rclnodejs/web/server` (server
  factory — `createRuntime` + transports), `rclnodejs/rosocket`.
  No deprecated aliases: the previous `cape` / `rclnodejs-runtime` /
  `rclnodejs/runtime` names from the in-development branch were
  removed before release because no version had ever shipped under
  them.
- **Typed SDK (single-string-generic)**: `ros.call<'pkg/srv/Name'>()`
  derives request + response shapes from the auto-generated
  `MessagesMap` / `ServicesMap` via the SDK's internal `WireType<T>`
  helper. Zero glue code or shared types module needed.
  ([web/index.d.ts](../web/index.d.ts))
- **`rclnodejs-web` CLI** (`rcl-web` short alias): declarative
  launcher so frontend developers don't have to write a Node.js
  server. Reads JSON config or accepts repeatable `--call` /
  `--publish` / `--subscribe` flags; supports ephemeral `--port 0`
  for tests. **Wires up the HTTP transport when `--http-port <n>`
  (or an `http: { port }` block in `web.json`) is supplied** — the
  banner reports both endpoints, and every exposed `call` / `publish`
  becomes reachable from `curl`, Postman, AI agents, etc. Exposed as
  a `bin` entry so `npx rclnodejs-web` works from a fresh install.
  ([bin/rclnodejs-web.js](../bin/rclnodejs-web.js),
  [test/test-web-cli.js](../test/test-web-cli.js))
- **Demos**:
  [`demo/web/javascript/`](../demo/web/javascript/)
  (no toolchain) and
  [`demo/web/typescript/`](../demo/web/typescript/)
  (Vite + tsx). Both are smoke-tested end-to-end.

### Still on the 2.0.0 docket

- [ ] **README surfacing**: add "Web" section to the main README
      between rosocket and Observable Subscriptions; the
      [`web/README.md`](../web/README.md) walkthrough is shipped —
      link it from the main README.
- [ ] **CI smoke test**: extend the existing test runner to cover the
      runtime path on Linux+Lyrical CI lanes.

### Known limitations carried forward to 2.1.x

- **SDK assumes the default path layout.** A single-URL `connect()`
  call hardcodes `/capability` as the basePath for both transports
  (`POST /capability/{call,publish}/<name>` for HTTP and
  `/capability` for the lazily-derived WebSocket sibling). If the
  server is started with a non-default `--http-base-path` / `--path`
  (or sits behind a path-rewriting proxy), callers must pass the
  fully-resolved URLs via the object form
  `connect({ http: 'https://host/api', ws: 'wss://host/cap' })`.
  Negotiated path discovery (a small `GET /capability/.well-known`
  on the server) is on the 2.1.x docket alongside reconnect.

## 2.1.0 — Production-readiness

**Round out the rough edges 2.0.0 users will hit first.** Two
usability deliverables and one observability win:

- [ ] **Action capability dispatch**. Replace the `not_implemented`
      reservation in the dispatcher with a real handler. WS is the
      required transport (goal/feedback/result is duplex by nature).
      Wire kind `'action'` is already reserved in 2.0.0 — no protocol
      bump needed.
- [ ] **Reconnect-on-close** in the Browser SDK. Today the
      `{reconnect: true}` option is accepted and warned-ignored; in 2.1.0
      it implements exponential backoff, automatic resubscribe, and
      `'reconnecting'` / `'reconnected'` events the application can hook
      into. Single biggest UX gap in 2.0.0.
- [ ] **Observability**: per-capability counters surfaced via a small
      `runtime.metrics()` call.

**Out of scope for 2.1.0**: schema validation, OpenAPI export, IAM,
SSE egress (the scoped SSE work is deferred to 2.2.0, since it depends
on Action capabilities proving their shape on WebSocket first).

## 2.2.0 — Schema validation, OpenAPI, name→type inference

**Turn the runtime into a documented Web API.** This is the release
that unlocks AI agent and code-generation use cases. It adds no new
_core_ protocol, but does extend the existing `HttpTransport` with an
SSE egress mode (write-only `Connection`) for the AI-agent / curl
personas — see "SSE: not for subscribe, yes for HTTP egress".

- [ ] **JSON Schema generation** from `.msg` / `.srv` / `.action`. Reuses
      the existing `rosidl_parser` AST. Cached in `generated/schemas/`.
- [ ] **Ingress + egress validation**: dispatcher runs Ajv against every
      payload; failures return `code: 'schema_violation'` with the JSON
      Pointer of the offending field. Strict mode opt-in per capability.
- [ ] **OpenAPI 3.1 export**: `runtime.openapi()` returns a complete
      spec — capability registry → paths, JSON Schemas → components.
      Routes are documented for both transports (WS frame envelope and
      HTTP endpoints).
- [ ] **CLI**: `rclnodejs-web openapi --config web.json > openapi.json`.
- [ ] **L3 typed SDK (name → type inference)**: build-step walks
      `runtime.expose({...})` and emits a project-local
      `.rclnodejs-web/capabilities.d.ts` so `ros.call('/navigate', req)`
      type-checks against the right ROS interface without manual generics.
- [ ] **SSE egress on `HttpTransport`** (scoped — _not_ a WS replacement;
      see "SSE: not for subscribe, yes for HTTP egress" below). Adds
      server→client streaming to the otherwise request/response HTTP
      transport, for the curl / AI-agent / edge-function personas that
      cannot hold a WebSocket open. Two narrow uses only:
  - **Action feedback over HTTP**: a goal started via
    `POST /capability/action/<name>` responds with `text/event-stream`
    — `feedback` events terminated by a single `result` event. This is
    the canonical LLM-style streaming pattern and directly serves the
    AI-agent thesis of this release. Depends on Action capabilities
    having proven their shape on WebSocket in 2.1.0.
  - **Opt-in HTTP-only `subscribe`**: `GET /capability/subscribe/<name>`
    returning `text/event-stream`, making topic streams curl-able and
    DevTools-visible for clients with no WS. Off by default; browser
    apps keep using the WS sibling.
    Implemented as a **write-only `Connection`** over the existing L2
    `TransportAdapter` seam: the dispatcher already emits
    `event: 'message'` envelopes, so the SSE adapter only reframes the
    same `{ok, error, code}` contract as `event:`/`data:` lines — no
    dispatcher protocol bump, no new wire codes beyond the already-reserved
    `streaming_unsupported`.

## SSE: not for subscribe, yes for HTTP egress

Two different questions hide under "should we use SSE?":

1. **Should SSE replace WebSocket as the `subscribe` path?** No —
   settled below, and not reopened.
2. **Should SSE add server→client streaming to the HTTP transport?**
   Yes, narrowly — scoped into 2.2.0 above (Action feedback over HTTP
   - opt-in HTTP-only subscribe).

### (1) SSE does not replace WebSocket for subscribe

Earlier drafts of this roadmap had an `SseTransport` standing in for
the WebSocket `subscribe` path to "complete" the HTTP transport story.
We deliberately rejected that framing. The argument:

| Concern                                            | WebSocket                             | SSE                                  |
| -------------------------------------------------- | ------------------------------------- | ------------------------------------ |
| Multiplexes N subscriptions on one connection      | ✅                                    | ❌ one HTTP request per topic        |
| Counts against browser per-origin connection limit | ❌ one socket                         | ⚠️ HTTP/1.1 caps you at 6 per origin |
| Wire efficiency for high-rate sensor topics        | ✅ binary frames                      | ⚠️ text + line-parse overhead        |
| Same channel for goal/feedback/result (Actions)    | ✅                                    | ❌ unidirectional                    |
| Auto-reconnect                                     | manual (we're shipping this in 2.1.0) | built-in to `EventSource`            |
| Curl-able subscribe                                | ❌                                    | ✅                                   |
| DevTools per-message visibility                    | ⚠️ inside the WS connection           | ✅ each event visible                |

The only places SSE clearly wins (auto-reconnect, curl-ability,
DevTools visibility) are either solved by other 2.1.0 work
(reconnect-on-close in the SDK) or are minor convenience wins.
WebSocket already does subscribe better in every operational
dimension that matters at scale.

We also couldn't identify a user persona that _needs_ SSE **for
subscribe** and can't use WS:

- Browser apps: already opening a WS for `call`/`publish`; sub on
  the same connection is free.
- CLI tools: `websocat` is a one-liner.
- AI agents: want discrete tool calls (HTTP `call`), not
  long-running topic streams.
- Behind-WS-blocking-proxy: vanishingly rare with `wss://` in 2026.

So WebSocket stays the default `subscribe` path. The HTTP-only
`subscribe` option in 2.2.0 is a convenience for clients with _no_ WS
at all (curl, observability), not a replacement for the browser path.

### (2) SSE _does_ fill the HTTP transport's egress gap

Today [`HttpTransport`](../lib/runtime/transports/http.js) is purely
request/response — `subscribe` and `action` return `unsupported_kind`,
so an **HTTP-only deployment has no server→client streaming at all**.
That is the real gap, and it hits exactly the personas this product
line is courting: AI agents, serverless/edge functions, and curl-based
integration that are HTTP-native and frequently _cannot_ hold a
long-lived WebSocket. EventSource is the standard, dependency-free way
to stream to them.

The two scoped uses (Action feedback over HTTP; opt-in HTTP-only
subscribe) are listed under 2.2.0. They are cheap because the L2
`TransportAdapter` seam already isolates Layer 3 from the wire: an SSE
stream is just a **write-only `Connection`** that reframes the
dispatcher's existing `event: 'message'` / `{ok, error, code}` envelope
as `event:`/`data:` lines. The wire _contract_ is unchanged; only the
_framing_ differs.

Honest constraints we will document rather than paper over:

- **Native `EventSource` is GET-only and header-less.** Browser
  clients can't `POST` a goal or send `Authorization` on the stream
  itself; action-over-SSE from a browser needs `fetch` +
  `ReadableStream`, or goal-via-`POST` then GET-stream-by-id. curl /
  agents have no such limit. Auth falls back to cookie or query token
  (the latter leaks in logs) until the 2.3.0 IAM work lands.
- **Text-only.** Image / pointcloud topics would require base64 bloat
  — those stay on WS (and CBOR in 2.3.0+). SSE is for control-plane
  and feedback streams, not high-rate sensor data.
- **Weaker backpressure than WS.** A fast topic on a slow SSE client
  needs a coalesce / drop / rate-cap policy or it balloons memory.
- **Per-origin connection cap** (6 on HTTP/1.1) only bites a browser
  opening many SSE streams; HTTP/2 multiplexing dissolves it, and the
  agent / curl persona opens a single stream anyway.

The L2 `TransportAdapter` interface stays open either way — anyone with
a need we haven't scoped can ship an out-of-tree adapter. We carry the
high-value egress slice first-party because it directly serves the
AI-agent thesis the 2.2.0+ releases are built around.

## 2.3.0+ — Production-grade

- [ ] **IAM scopes**: `runtime.scope('viewer', { subscribe: ['/scan'] })`
  - JWT verifier hook + per-connection scope selection.
- [ ] **Pluggable auth providers**: built-ins for static token, JWT, and
      mTLS; `AuthProvider` interface for OAuth/OIDC.
- [ ] **MCP server**: thin adapter over the capability registry exposing
      every capability as an MCP tool. With L3 typed SDK already shipped,
      this is mostly a 200-LOC surface.
- [ ] **CBOR / binary frame support** for large messages (image,
      pointcloud) — keeps JSON as the default but adds opt-in compression.
- [ ] **QoS surfacing** through the registry:
      `expose({ subscribe: { '/scan': { type: '...', qos: 'sensor_data' }}})`.

## Wire protocol stability

The wire format frozen in 2.0.0 is what Browser SDKs and external
clients depend on. Future minor releases extend it; we will not break
2.0.0 clients without a major bump:

| Field                   | Type                                                      | Stability                                      |
| ----------------------- | --------------------------------------------------------- | ---------------------------------------------- |
| `id`                    | string                                                    | stable — opaque to runtime                     |
| `kind`                  | `'call'\|'publish'\|'subscribe'\|'unsubscribe'\|'action'` | stable; new kinds reserved by extension        |
| `capability`            | string (ROS name)                                         | stable                                         |
| `payload`               | any JSON value                                            | stable; semantics defined by capability type   |
| `subId`                 | string                                                    | stable — used for unsubscribe + sub deliveries |
| `ok` / `error` / `code` | reply contract                                            | stable; codes additive                         |
| `event: 'message'`      | sub delivery envelope                                     | stable                                         |

## Cross-references

- [WEB_RUNTIME_ARCH.md](./WEB_RUNTIME_ARCH.md) — architecture & rationale.
- [RCLNODEJS_WEB_VS_ROSBRIDGE.md](./RCLNODEJS_WEB_VS_ROSBRIDGE.md) —
  differentiation vs. rosbridge / roslibjs.
- [DISCOURSE_2.0.0-beta.0.md](./DISCOURSE_2.0.0-beta.0.md) — public
  framing for the beta release (May 2025).
- [DISCOURSE_2.0.0.md](./DISCOURSE_2.0.0.md) — announcement draft for
  the GA release.
