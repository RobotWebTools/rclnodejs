# `rclnodejs/web` vs `roslibjs` + `rosbridge_server`

> Differentiation tracking document. Captures the real (and only the
> real) differences between the new web stack shipping inside
> rclnodejs and the long-established `rosbridge_server` + `roslibjs`
> combo. Used as the source of truth for marketing copy, README
> blurbs, and Discourse posts so we don't overclaim.
>
> Written May 2026, before the 2.0.0 announcement. Maintainer-facing;
> not shipped to the npm tarball.

## TL;DR

`rclnodejs/web` is a typed Web SDK and **capability runtime** for ROS 2,
built into rclnodejs. The wire transport is the same as rosbridge
(WebSocket + JSON), but the API contract is fundamentally different:
the server publishes a **declarative allow-list** of capabilities
(`web.json`), not the whole ROS graph. Browser code is typed end to
end via single-string generics that derive from rclnodejs's
auto-generated `MessagesMap` / `ServicesMap`. Every `call` and
`publish` is also reachable from plain HTTP, so AI agents and
curl-based scripts drive the robot without a JavaScript stack.

> **Framing.** This is **not a new robot SDK**. It's modern web
> infrastructure for the ROS graph that already exists. The browser
> still talks to topics and services; what changed is the contract,
> the type system, and the operational surface around them. See
> [§“Why we don't try to hide the ROS graph”](#why-we-dont-try-to-hide-the-ros-graph)
> for the rationale.

## Side-by-side

```js
// rosbridge + roslibjs
const ros = new ROSLIB.Ros({ url: 'ws://host:9090' });
const topic = new ROSLIB.Topic({
  ros,
  name: '/chatter',
  messageType: 'std_msgs/String',
});
topic.subscribe((msg) => console.log(msg.data));
```

```ts
// rclnodejs/web
import { connect } from 'rclnodejs/web';
const ros = await connect('ws://host:9000/capability');
const reply = await ros.call<'example_interfaces/srv/AddTwoInts'>(
  '/add_two_ints',
  { a: '7n', b: '35n' }
);
```

Same job. Different contract, different ergonomics, different
operational surface.

## The five real differences

These are the differences that hold up under questioning. **If a
marketing claim can't be derived from one of these five, we don't
make the claim.**

### 1. Allow-list vs. open graph (security model)

|                                | `rosbridge` + `roslibjs`                         | `rclnodejs/web`                                             |
| ------------------------------ | ------------------------------------------------ | ----------------------------------------------------------- |
| Default                        | Any client can publish to any topic[^rb-auth]    | Capability must be in `expose({...})`                       |
| Failure mode for unknown topic | Pub/sub created on-the-fly                       | Rejected with `code: 'not_exposed'` before any ROS API runs |
| What the public API surface is | The whole ROS graph                              | The JSON file on disk                                       |
| Audit story                    | "What can the browser do?" → enumerate the graph | "What can the browser do?" → `cat web.json`                 |

[^rb-auth]: rosbridge ships optional authentication / `securityglobs` plugins that can gate clients, but there is no first-class declarative allow-list — the contract is configured per-deployment in launch files rather than published as a reviewable artifact.

**This is the architectural difference.** Everything else is downstream of it.

> Real-world consequence: with rosbridge, exposing a robot to the
> public internet means exposing every topic the robot publishes —
> including ones a future driver might add without anyone reviewing
> the web surface. With `rclnodejs/web`, the web surface is a static
> artifact that flows through code review.

### 2. Typed SDK derived from generated types (developer ergonomics)

|                             | `rosbridge` + `roslibjs`                                                                     | `rclnodejs/web`                                                                                                                                 |
| --------------------------- | -------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------- |
| TypeScript types            | `ROSLIB.Message` is `any`-shaped; community packages bolt types on with hand-written `.d.ts` | Single string generic: `ros.call<'pkg/srv/Name'>(...)` derives request + response from rclnodejs's auto-generated `MessagesMap` / `ServicesMap` |
| Type drift on `.msg` change | Silent; runtime error                                                                        | Compile error                                                                                                                                   |
| Setup needed                | Install + maintain a separate types package                                                  | None — rclnodejs already generates the types for the Node side; we reuse them in the browser                                                    |

> Real-world consequence: refactoring a `.srv` definition is a build
> error in IDE, not a 2 a.m. page. Python clients can't offer this
> end-to-end without a separate type-generation pipeline; rclnodejs
> already generates the types for the Node side, so reusing them in
> the browser was the obvious next step.

### 3. HTTP transport (curl-ability, AI agents)

|                                          | `rosbridge` + `roslibjs`                          | `rclnodejs/web`                                               |
| ---------------------------------------- | ------------------------------------------------- | ------------------------------------------------------------- |
| Service call from curl                   | ❌ — no HTTP surface                              | ✅ `curl -X POST .../capability/call/<name>`                  |
| Service call from a Postman collection   | ❌                                                | ✅                                                            |
| LLM agent calling the robot via tool-use | Needs custom WS shim                              | Plain HTTP `POST` → tool-use just works                       |
| Subscribe from curl                      | ❌ (rosbridge's WS protocol isn't shell-friendly) | ❌ (we deliberately stayed on WS for subscribe; SSE rejected) |

> Real-world consequence: as soon as someone wants to wire a service
> call into Zapier, n8n, an LLM tool definition, a CI smoke test, or
> a shell script — rosbridge requires writing a WS client.
> `rclnodejs/web` is one `curl`.

### 4. Zero backend code (DX)

|                            | `rosbridge` + `roslibjs`                                                                              | `rclnodejs/web`                                       |
| -------------------------- | ----------------------------------------------------------------------------------------------------- | ----------------------------------------------------- |
| Setup                      | `apt install ros-<distro>-rosbridge-server` + `roslaunch rosbridge_server rosbridge_websocket.launch` | `npm install rclnodejs && npx rclnodejs-web web.json` |
| Backend customization      | A Python launch file you usually don't touch                                                          | A JSON file you write                                 |
| Where the allow-list lives | Doesn't exist                                                                                         | The same JSON file                                    |

> Real-world consequence: a frontend-only team doesn't need a Python
> toolchain or a ROS workspace to put up a robot UI. They need
> Node.js (which they have) and a JSON file.

### 5. Wire protocol shape (RPC vs. graph operations)

|                                          | `rosbridge` + `roslibjs`                                                         | `rclnodejs/web`                                                                                                      |
| ---------------------------------------- | -------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------- |
| Op model                                 | `op: 'subscribe' \| 'publish' \| 'call_service' \| 'advertise' \| ...` (10+ ops) | `kind: 'call' \| 'publish' \| 'subscribe' \| 'unsubscribe' \| 'action'` (5)                                          |
| `advertise` step required before publish | Yes                                                                              | No (the allow-list already advertised it)                                                                            |
| RPC reply correlation                    | `id` field, present on most ops                                                  | `id` field on every WebSocket frame (UUID v4); HTTP is one-shot and elides it (the server synthesises a placeholder) |
| Error format                             | Mixed (some ops return `level: 'error'`, some don't)                             | Uniform `{ok: false, error, code}` with stable codes                                                                 |

> Real-world consequence: writing a non-`roslibjs` client (e.g. for a
> CI smoke test, or for a non-JS language, or for an MCP server) is
> materially smaller. Half the surface area, fewer state machines,
> structured errors.

## Why we don't try to hide the ROS graph

The most common pushback to anything in the `rclnodejs/web` family is
some form of:

> "If the browser still calls `ros.subscribe('/scan')` and
> `ros.call('/navigate')`, isn't this just modernised `roslibjs`?"

Fair question, and the answer matters because it constrains every
future design decision. Short version: **for a general-purpose ROS 2
web stack, we deliberately keep the ROS graph visible.** A truly
"web-native" abstraction — `robot.navigate({ room: 'kitchen' })`,
`robot.pickAndPlace(...)` — cannot be standardised across robots,
because robotics has no equivalent to the web's stable primitives
(URL, DOM, REST resource, SQL row). Any attempt to define one ends in
**capability explosion**: each robot, each fleet, each integrator
adds verbs, and the "abstraction" becomes an unbounded
product-specific API.

rosbridge/`roslibjs` made the same call ten years ago for the same
reason. Their failure mode isn't the graph contract; it's the
operational surface around it (no allow-list, no schema, no auth
story, no introspection, JSON-only transport, no HTTP path). That's
where `rclnodejs/web` differentiates — not by inventing a new
frontend abstraction, but by upgrading the **runtime** that exposes
the graph:

| Layer                | rosbridge + roslibjs       | `rclnodejs/web`                                             |
| -------------------- | -------------------------- | ----------------------------------------------------------- |
| Frontend abstraction | ROS graph (topic/service)  | ROS graph (topic/service) — **deliberately the same**       |
| Public API contract  | Implicit; whole live graph | Explicit; `web.json` allow-list                             |
| Browser type system  | `any` + community stubs    | Single-string generic over generated `MessagesMap`          |
| Non-JS clients       | Custom WS shim required    | Plain HTTP `POST /capability/call/<name>` (curl, AI tools)  |
| Auth / governance    | `securityglobs` plugin     | Per-transport hooks today; per-capability scopes on roadmap |
| Operational story    | Python launch file         | `npx rclnodejs-web web.json`                                |

Product-specific abstractions (`robot.navigate()`, `robot.dock()`)
belong to the **integrator**, layered on top of the graph contract —
not to a general-purpose runtime. That boundary is what keeps the
project from drifting into "another robot SDK" territory.

> Bottom line: `rclnodejs/web` is **infrastructure for the existing
> ROS graph**, not a replacement contract for the browser. Honest
> framing protects the differentiation; overclaiming a hidden-graph
> abstraction would set up a feature-parity argument we'd lose.

## Differences we will **not** claim

These claims are tempting but false (or unprovable today). Do not let
them slip into copy.

| ❌ Claim                      | Why we don't say it                                                                                                                                                        |
| ----------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| "Faster than rosbridge."      | We use WebSocket + JSON, same as rosbridge. Until CBOR ships, throughput is comparable.                                                                                    |
| "More robust than rosbridge." | rosbridge has 10+ years of production hardening. We don't. Hard truth.                                                                                                     |
| "Replaces rosbridge."         | It doesn't. Different user, different contract. rosbridge is correct when you want graph-shaped semantics (rqt-web, fleet dashboards that genuinely need topic discovery). |
| "Web-native ROS 2."           | Not yet. Earned when OpenAPI export + schema validation ship.                                                                                                              |

## When to choose which

A frontend dev evaluating this should reach for `rclnodejs/web` when
**at least one** of these is true:

1. They want **types in their IDE** without setting up a separate
   type-generation pipeline.
2. They want their robot's web surface to be **a reviewable artifact**
   (a JSON file), not an open graph.
3. They want **HTTP-callable endpoints** for AI agents, curl, Postman,
   CI, Zapier, etc.
4. Their team has Node.js skills but not Python/ROS skills, and the
   operator is asking them "can you put a button on the robot's web
   page?"

A frontend dev should **stick with `rosbridge` + `roslibjs`** when:

1. They have an existing `roslibjs` codebase and migration cost >
   benefit.
2. They genuinely need to introspect the ROS graph from the browser
   (debug consoles, `rqt`-style tools).
3. They need a feature rosbridge has and we don't yet (Actions are
   the obvious one for now).

## The one-paragraph version (announcement-ready)

> **`rclnodejs/web` is a typed Web SDK and capability runtime for
> ROS 2, built into rclnodejs.** Unlike `rosbridge` + `roslibjs`, the
> server publishes a declarative allow-list of capabilities
> (`web.json`), not the whole ROS graph; the browser SDK derives
> request, response, and message types from rclnodejs's
> auto-generated TypeScript definitions, so a single string generic
> gets you full type-checking with no glue code; and every `call` and
> `publish` is reachable from plain HTTP, so AI agents and curl-based
> scripts drive the robot without a JavaScript stack. Subscribe stays
> on WebSocket. Setup is `npx rclnodejs-web web.json` — no Python, no
> ROS workspace.

This paragraph is the benchmark for everything else. **If a sentence
elsewhere in our copy can't be derived from it, that sentence is
overclaiming.**

## Things to _not_ say in 2.0.0 announcements

- ❌ "Web-native ROS 2 runtime" — implies multi-transport / OpenAPI
  story we won't ship until 2.2.0.
- ❌ "Replaces rosbridge / roslibjs" — invites a feature-parity
  argument we don't want.
- ❌ "Backend-for-Frontend layer for ROS 2" — too abstract for the
  size we've shipped.

## Things to say

- ✅ "Typed Web SDK for ROS 2." (true and demonstrable in 10 lines.)
- ✅ "Capability allow-list — your browser only sees what you
  `expose({...})`." (true and architectural.)
- ✅ "No backend code: `npx rclnodejs-web web.json`." (true and a
  concrete win.)
- ✅ "WebSocket and HTTP transports today; OpenAPI export and Actions
  in 2.1 / 2.2." (sets the trajectory honestly.)
- ✅ "`curl -X POST .../capability/call/...` works." (concrete proof
  of the HTTP path.)

## Cross-references

- [WEB_RUNTIME_ARCH.md](./WEB_RUNTIME_ARCH.md) — architecture &
  layered design (the "why" behind the allow-list contract).
- [WEB_RUNTIME_ROADMAP.md](./WEB_RUNTIME_ROADMAP.md) — per-release
  roadmap, "Why no SSE?", wire-protocol stability.
- [`rosocket`](../rosocket/) — sibling, **not** competitor. Same
  monorepo, different contract: `rosocket` is a thin per-resource
  WebSocket gateway (`/topic/<name>`, `/service/<name>`) for quick
  prototypes and `roslibjs`-style apps; `rclnodejs/web` is the typed
  capability runtime described in this document. Run them as
  separate processes on separate ports when both are needed.
  See [`web/README.md`](../web/README.md) for the public
  comparison.
