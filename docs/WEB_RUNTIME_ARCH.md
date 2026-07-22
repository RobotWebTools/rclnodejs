# rclnodejs as a Web Runtime for ROS 2 — Architecture & Roadmap

> Strategic evaluation of how rosocket (shipped in `2.0.0-beta.0`)
> evolves into a layered Web Runtime that differentiates rclnodejs
> from rclpy and rclcpp. Written May 2026, alongside the `2.0.0` GA.
>
> **Honest scope.** This doc lays out the _eventual_ architecture
> through 2.3.0+. The "web-native runtime" framing is **not** what
> we claim in 2.0.0 — see [WEB_RUNTIME_ROADMAP.md](./WEB_RUNTIME_ROADMAP.md)
> ("What we are claiming, and what we aren't") for the per-release
> framing. 2.0.0 ships the **WebSocket and HTTP transports** plus the
> typed SDK and CLI; schema validation, OpenAPI export, and the
> remaining web-native pieces are earned over 2.1.x and 2.2.x.

## 1. The two-layer framing

The current rosocket is a **transport-level gateway**. The headroom for
differentiation is in the **capability/runtime layer** above it:

- **rclpy/rclcpp** will never be the BFF (Backend for Frontend) for
  browsers — Node.js has the only mature web stack inside the ROS 2
  ecosystem. This is rclnodejs's real moat, not "another binding."
- **Capability-oriented APIs > graph-oriented APIs** for browsers. A
  frontend developer wants `await ros.call('/navigate', {target:'kitchen'})`,
  not "create a service client, wait for service to become available,
  send a goal, await the future." Hiding ROS 2 mechanics behind a Web
  SDK is the differentiator.
- **Auto-generated TypeScript contracts are a real moat.** rclnodejs
  already generates `.d.ts` for every interface — we're already 50% of
  the way to typed RPC SDKs. rclpy literally cannot do this.

## 2. Where rosocket stands today (commit `3176aca`)

What's in:

- Resource-style URLs (`/topic/<name>`, `/service/<name>`) — already
  cleaner than rosbridge's op-coded JSON.
- Optional pre-declared `topicTypes` / `serviceTypes` map — first sketch
  of a "capability registry."
- `verifyClient` hook — first authn surface.
- `BigInt` round-tripping — handles the 64-bit pain point most bridges
  ignore.

What's missing:

- No schema enforcement — browser can send arbitrary JSON.
- No actions, no QoS, no introspection.
- No client SDK — every browser caller hand-rolls `WebSocket + JSON.parse`.

It's a solid **transport**. It is not yet a **runtime**.

## 3. Proposed architecture (4 layers, ship-incrementally)

```
┌─────────────────────────────────────────────────────────────┐
│  Layer 4: Browser SDK (rclnodejs/web)                       │  ← user-facing
│    ros.call(), ros.publish(), ros.subscribe(),              │
│    ros.action(), typed via generated .d.ts                  │
├─────────────────────────────────────────────────────────────┤
│  Layer 3: Capability Runtime                                │  ← differentiation
│    - capability registry (declarative allow-list)           │
│    - typed contracts (generated from .msg/.srv/.action)     │
│    - auth/IAM (per-capability scopes)                       │
│    - schema validation (ingress + egress)                   │
│    - introspection endpoint (OpenAPI + JSON Schema)         │
├─────────────────────────────────────────────────────────────┤
│  Layer 2: Transport Adapters                                │  ← pluggable
│    rosocket (WS)  │  HTTP RPC  │  (gRPC?)  │  (extensible)  │
├─────────────────────────────────────────────────────────────┤
│  Layer 1: rclnodejs Core (today, unchanged)                 │
│    Node, Topic, Service, Action, Parameter, Lifecycle       │
└─────────────────────────────────────────────────────────────┘
```

### Layer responsibilities

**Layer 1 — rclnodejs core.** No change. This is the asset.

**Layer 2 — Transport adapters.** Today both **WebSocket** and **HTTP**
adapters ship as first-party (`/capability` for WS, `POST
/capability/call/<name>` and `POST /capability/publish/<name>` for
HTTP). Subscriptions stay on the WebSocket transport — see the
"Why no SSE?" section in [WEB_RUNTIME_ROADMAP.md](./WEB_RUNTIME_ROADMAP.md).
Each adapter implements one interface (`TransportAdapter` +
`Connection`); Layer 3 only sees that interface. The seam is what
lets future adapters (gRPC, CBOR-over-WS, an out-of-tree SSE) drop
in without touching the runtime or the SDK.

**Layer 3 — Capability Runtime.** This is the new code. A `capabilities`
config (file or fluent API) declares what the browser is allowed to do:

```js
runtime.expose({
  call: { '/navigate': 'nav2_msgs/srv/NavigateToPose' },
  publish: { '/cmd_vel': 'geometry_msgs/msg/Twist' },
  subscribe: { '/scan': 'sensor_msgs/msg/LaserScan' },
  action: { '/move': 'nav2_msgs/action/NavigateToPose' },
});
```

Three side effects fall out naturally:

1. **Schema validation** — runtime knows the type, can ajv-validate every
   ingress/egress message.
2. **IAM/scopes** — `runtime.scope('viewer', { subscribe: ['/scan'] })`
   then validate JWT claim.
3. **Introspection** — `GET /capabilities` returns OpenAPI 3.1 + JSON
   Schema generated from the registered types. This is the artifact that
   turns rclnodejs into a _web API_.

**Layer 4 — Browser SDK** (`rclnodejs/web` subpath import). One thin
client, transport-aware:

```ts
import { connect } from 'rclnodejs/web';

// WebSocket only
const ros = await connect('ws://robot.local:9000/capability');

// or HTTP for call/publish + WS sibling for subscribe
const ros2 = await connect({
  http: 'http://robot.local:9001',
  ws: 'ws://robot.local:9000/capability',
});

const result = await ros.call<'nav2_msgs/srv/NavigateToPose'>('/navigate', {
  /* typed request */
});
const sub = await ros.subscribe<'sensor_msgs/msg/LaserScan'>('/scan', (scan) =>
  render(scan)
);
```

The `.d.ts` for every ROS interface comes from the existing
`generate-ros-messages` pipeline — **rclnodejs already does this for
Node**. The SDK reuses the generated `MessagesMap` / `ServicesMap`
so a single string generic gives you the request, response, and
message shapes for free.

## 4. Phased delivery

| Phase                      | Deliverable                                                                                                                         | Why                                                                                      |
| -------------------------- | ----------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------- |
| **2.0.0-beta.0** (shipped) | Ship rosocket as-is. Headline: "WebSocket bridge to ROS 2."                                                                         | It works, it's lean, no overpromising.                                                   |
| **2.0.0** (shipping)       | `runtime.expose({...})` declarative capability registry, typed Browser SDK, **WebSocket and HTTP transports**, `rclnodejs-web` CLI. | First vertical slice through all four layers. The L2 seam is proven with two adapters.   |
| **2.1.0**                  | Action capabilities, reconnect-on-close in the SDK, observability counters.                                                         | Round out the production-readiness gaps.                                                 |
| **2.2.0**                  | JSON Schema validation, OpenAPI 3.1 export from the registry, name→type inference for the SDK.                                      | Turns rclnodejs into a real web framework. AI agents and code-gen tools become possible. |
| **2.3.0+**                 | Per-capability IAM/scopes, MCP server, CBOR.                                                                                        | Production-grade.                                                                        |

## 5. Differentiation vs rclpy / rclcpp

After this:

| Capability                             | rclcpp |       rclpy       | rclnodejs (with runtime) |
| -------------------------------------- | :----: | :---------------: | :----------------------: |
| ROS 2 client (topics/services/actions) |   ✅   |        ✅         |            ✅            |
| TypeScript types from `.msg`/`.srv`    |   ❌   |        ❌         |            ✅            |
| Browser SDK                            |   ❌   |        ❌         |            ✅            |
| HTTP / WS transports                   |   ❌   | partial via Flask |            ✅            |
| OpenAPI export                         |   ❌   |        ❌         |            ✅            |
| Capability/scope-based IAM             |   ❌   |        ❌         |            ✅            |
| Performance-critical control           |  ✅✅  |        ✅         | ✅ (but not the target)  |

That last row is honest: **rclnodejs is not trying to be the realtime
control library**. It's trying to be the **web/cloud/HMI plane** of a
ROS 2 system. That's a defensible niche where neither rclcpp nor rclpy
can compete.

## 6. Caveats / things to push back on

A few claims from the source discussion that look strong on paper but
don't survive contact with reality:

- **"HTTP RPC + SSE for everything, WebSocket only as fallback" is too
  dogmatic.** It's a defensible _default_ policy for `call` semantics,
  not an architectural law for _everything_. ROS 2 services and Actions
  are bidirectional with goals/feedback/cancellation; subscriptions
  multiplex many topics on a single connection — modeling that as one
  SSE stream per topic burns the browser's per-origin connection
  budget for no real benefit. The honest answer is **transport
  per concern**: HTTP for `call`/`publish`, WS for `subscribe` and
  Actions. We deliberately do not ship SSE — see the "Why no SSE?"
  section in [WEB_RUNTIME_ROADMAP.md](./WEB_RUNTIME_ROADMAP.md).
- **"Don't expose the ROS graph to the browser" is overstated.** Some
  real apps (rqt-style tools, fleet dashboards, debug consoles)
  genuinely need graph introspection. The right move is _layered_:
  capability API on top, graph API still available underneath.
- **OpenAPI/JSON Schema for ROS 2 only pays off** if there's a
  `ros 2 web ...` CLI generator, hot reload, server-side type checks,
  and round-trip schema validation. Without that pipeline, it's just
  YAML.
- **rosocket should not be rebranded "down" to plugin status today.**
  It's the only working artifact in `2.0.0-beta.0` and deserves to be
  the headline. The runtime layer is months away.

## 7. Recommendations (TL;DR)

1. **Adopt the layered architecture as a code structure**, not a
   manifesto. Translate it into the actual `lib/runtime/` directory for
   2.x.
2. **Don't rewrite rosocket. Don't rebrand it.** Keep it as the realtime
   adapter. Build the runtime on top.
3. **The single most leveraged next deliverable is the capability
   registry + schema validation** — small code, big positioning shift.
   ~200 LOC + tests on top of rosocket.
4. **The OpenAPI generator is the killer feature**, but only after the
   registry exists. Don't try to do it first.
5. **Position rclnodejs as "the Node.js client for ROS 2 + a typed
   web SDK and capability runtime,"** not "rosbridge done right." The
   latter invites comparison with a 10-year-old project; the former
   defines a new category.
6. **An MCP server on top of the capability runtime** would let LLM
   agents drive a robot via the same registry. With the typed contracts
   already in place, this is almost free, and it's the AI-agent story
   the ecosystem is asking for.

## 8. Related

- `rosocket/README.md` — transport-layer implementation.
- `docs/WEB_RUNTIME_ROADMAP.md` — per-release roadmap, error-code
  table, "Why no SSE?".
- `docs/RCLNODEJS_WEB_VS_ROSBRIDGE.md` — differentiation vs.
  rosbridge / roslibjs.
- `docs/DISCOURSE_2.0.0-beta.0.md` — public framing for the beta
  release (May 2025).
- `docs/DISCOURSE_2.0.0.md` — announcement draft for the GA release.
