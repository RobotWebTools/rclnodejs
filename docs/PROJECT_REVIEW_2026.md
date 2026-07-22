# rclnodejs Overall Project Review (2026)

Status: working notes / proposal — not a commitment.
Author: code review pass requested June 2026.
Scope: a holistic review of the `rclnodejs` package from two angles:

1. **Parity with the official ROS 2 Python client (`rclpy`)** — API surface, executor
   model, and behavior.
2. **Web / JavaScript ecosystem health** — module system, ES standard adoption,
   TypeScript story, tooling, and the new browser (`web/`) runtime.

This document is intended as a living tracker. It deliberately avoids duplicating the
per-feature checklist in [RCLPY_PARITY_STATUS.md](RCLPY_PARITY_STATUS.md); instead it
takes a step back and groups findings into themes with a recommended priority.

---

## 1. Executive summary

rclnodejs is in good shape. The codebase already uses modern ES6+ idioms (classes,
`const`/`let`, arrow functions, destructuring, spread, getters/setters, `AbortSignal`),
ships first-class TypeScript declarations, supports prebuilt N-API binaries, and tracks
rclpy parity carefully. The recent additions (web runtime, QoS overriding, parameter
event handler, `MessageInfo`, logger service) show the project is actively closing gaps
with rclpy.

The biggest opportunities are **not** missing ROS features — those are mostly tracked and
mostly intentional skips. The biggest opportunities are **developer-experience and
ecosystem modernization**:

- The library is **CommonJS-only**. Modern Node/web tooling increasingly assumes ESM.
- The async story is **split between callbacks and Promises**; an async-first surface
  (and optionally async iterators) would feel more idiomatic to today's JS developers.
- The TypeScript story is **hand-written `.d.ts` against `target: es2020`**, which lags
  the `engines.node >= 20` runtime (which is ES2023-capable).
- The **executor / spin model** differs from rclpy in ways worth documenting explicitly
  so users coming from Python aren't surprised.

The rest of this doc expands each theme.

---

## 2. rclpy parity review

### 2.1 What's already solid

The mapping to rclpy core entities is essentially complete:

| rclpy concept                         | rclnodejs equivalent                            | Notes                                              |
| ------------------------------------- | ----------------------------------------------- | -------------------------------------------------- |
| `Node`, `Context`                     | `Node`, `Context`                               | Multi-context supported                            |
| `Publisher`, `Subscription`           | `Publisher`, `Subscription`                     | Raw msgs, event callbacks, content filter          |
| `Client`, `Service`                   | `Client`, `Service`                             | Callback **and** Promise (`sendRequestAsync`)      |
| `Timer`, `Rate`                       | `Timer`, `Rate`                                 | `autostart`, `TimerInfo` injection                 |
| `GuardCondition`                      | `GuardCondition`                                |                                                    |
| Parameters (declare/get/set/describe) | Parameters                                      | Pre/post/on-set callbacks, `ParameterEventHandler` |
| QoS profiles + overriding             | `QoS`, `QoSOverridingOptions`                   |                                                    |
| Clock / Time / Duration / TimeSource  | `Clock`, `Time`, `Duration`, `TimeSource`       | ROS time, jump handlers                            |
| Lifecycle nodes                       | `lifecycle.js`, `lifecycle_publisher.js`        |                                                    |
| Actions (client/server)               | `lib/action/`                                   | Goal state guards, feedback filter                 |
| Logging + logger service              | `logging.js`, `logging_service.js`              | `enableLoggerService`                              |
| Type description service              | `type_description_service.js`                   |                                                    |
| `wait_for_message`                    | `waitForMessage()`                              |                                                    |
| Serialization                         | `serialization.js` / `message_serialization.js` | Multiple modes                                     |

rclnodejs also has **features rclpy lacks** (RxJS `ObservableSubscription`, message
validation/schemas, `MessageIntrospector`, `ros2Run`/`ros2Launch`, JSON-safe conversion).

### 2.2 Intentional divergences (document these clearly)

These are reasonable JS-specific choices, but they surprise users migrating from Python.
Recommendation: add a short **"Coming from rclpy"** page to the docs that states each one.

- **No `MultiThreadedExecutor` / callback groups.** JS is single-threaded; libuv provides
  concurrency and callbacks are already serialized. (Tracked as "skipped".)
- **No `EventsExecutor`.** Experimental in rclpy.
- **No `spinUntilFutureComplete`.** In JS the idiom is `node.spin(); await future; node.stop();`
  or simply `await client.sendRequestAsync(...)`. Worth a one-paragraph recipe.
- **Signal handling** relies on libuv's immediate SIGINT delivery rather than a signal
  guard condition.

### 2.3 Genuine gaps worth re-evaluating

These are low priority but listed so they don't get lost:

- **`Waitable` extensibility interface** — lets advanced users add custom wait-set
  entities. High effort, low demand. Keep on backlog.
- **Subscription buffer backends** (`acceptable_buffer_backends`) — no JS/native interop
  story yet. Keep on backlog.
- **rclpy experimental async API** (`AsyncNode`, `async for msg in sub`, `await
pub.publish_async()`). This is _interesting for rclnodejs_ because JS already has native
  async iteration — see §3.3. This is the one rclpy direction that maps very naturally to
  JS and could become a differentiator rather than a copy.

### 2.4 Recommendation

Parity is effectively "done" for practical purposes. Stop chasing 1:1 parity and instead
**lean into JS-native ergonomics** (async iterators, `Symbol.asyncIterator`, `AbortSignal`
everywhere). The parity tracker can move to maintenance mode.

---

## 3. Web / JavaScript modernization review

### 3.1 Module system: CommonJS → consider dual ESM

**Current state:** Pure CommonJS (`require` / `module.exports`) across `lib/` and
`index.js`. Only `eslint.config.mjs` is ESM. The `package.json` `exports` map is modern
and already supports conditional subpaths (`.`, `./web`, `./web/server`, `./lib/*`).

**Why it matters:** ESM is the default authoring format for new Node and all browser/
bundler tooling (Vite, esbuild, Webpack 5, Rollup). CJS-only packages force `await import()`
interop, break tree-shaking, and feel dated. Node 20 (the minimum here) fully supports ESM,
and recent Node versions even allow `require()` of ESM.

**Existing work — this is already planned and underway.** A detailed migration plan lives in
[issue #1358](https://github.com/RobotWebTools/rclnodejs/issues/1358) ("ES Module Migration
Plan for rclnodejs"), itself a child of the [#903 "Adopt ES2020 features"](https://github.com/RobotWebTools/rclnodejs/issues/903)
epic. The maintainers (@mahmoud-ghalayini, @minggangw) have **agreed on an ESM-first authoring
approach with a `tsup` bundler** that emits both ESM and CJS, so existing
`const rclnodejs = require('rclnodejs')` consumers keep working with no breaking change.

**Agreed plan and current status:**

| Phase | Scope                                                                                                                                                                                                                            | Status                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| ----- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 1     | Rename CJS-only files (`scripts/*`, then `rosidl_gen/*`, `rosidl_parser/*`, `rostsd_gen/*`) to `.cjs`                                                                                                                            | ✅ done on branch `esm-migration-phase1-3` (all four dirs renamed, refs + emitted template paths updated, verified via native build + full message regen + runtime load test). `scripts/*` originally via [PR #1360](https://github.com/RobotWebTools/rclnodejs/pull/1360)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 2     | Convert all ~50 `lib/*.js` + `index.js` to ESM (atomic): drop `'use strict'`, `require`→`import`, `module.exports`→`export`; `createRequire()` for native loader; dynamic `import()` for the `rate.js` ↔ `index.js` circular dep | ✅ done on branch `esm-migration-phase2`. All `lib/**/*.js` (incl `action/`, `runtime/`), `index.js`, `bin/rclnodejs-web.js`, `rosocket/*` and all `test/*.js` converted; folded in `"type": "module"` + `generated/package.json` `{"type":"commonjs"}` CJS-island guard (generator emits it too). Generator/parser `.cjs` that `require()` now-ESM lib modules unwrap `.default`. `eslint.config.mjs` split ESM vs CJS `sourceType`. Verified: native build + full message regen + full mocha suite (**155 passing**; 1 pre-existing `setInterval`-leak flake in `test-type-description-service.js`, passes in isolation) + eslint 0 errors                                                                                                                                                                                                                                                                                                          |
| 3     | Add dual `exports` (`import`/`require` conditions), `tsup.config.js`, `dist/` build (`"type": "module"` already added in Phase 2)                                                                                                | ✅ done on branch `esm-migration-phase3`. `tsup.config.js` dual-emits ESM+CJS for `.`, `./web`, `./web/server`, `./rosocket` into a flat `dist/` (depth-1 keeps the native loader's `..`-relative `prebuilds/`/addon paths correct). Generator islands (`rosidl_gen`/`rostsd_gen`) are externalized and path-rewritten relative to `dist/`. A per-format footer hoists the lone `export default` to `module.exports` so `require('rclnodejs')` returns the value directly (needed since `engines.node >= 20.20.2` predates `require(ESM)`). `exports`/`main`/`module` repointed to `dist/`; `./lib/*` stays ESM-only; `dist/` un-ignored for npm, built via `build:dist`/`prepack`. `web/client.js` lazy-loads `ws` (removed a top-level `await` incompatible with the CJS output). Verified: dual build + `require()`/`import` smoke tests + functional init/pub/sub/shutdown via the CJS bundle + 56 web tests + full mocha suite + eslint 0 errors |
| 4     | Update the 36 `types/*.d.ts` declarations for ESM                                                                                                                                                                                | not started                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 5     | (optional) convert generator/parser `.cjs` modules back to ESM                                                                                                                                                                   | not started                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |

**Implications that supersede earlier framing:**

- The team chose the **bundler (tsup) path**, not a thin hand-written ESM wrapper — it
  preserves _synchronous_ `require()` backward compatibility, which a `createRequire`/proxy
  wrapper cannot.
- This means **`"type": "module"` will be added** in Phase 3 (so bare `.js`/`.ts` become ESM),
  with the CJS half emitted as `dist/index.cjs`.
- A known constraint from the discussion: **in-repo tests must switch** from
  `require('../index.js')` to `import rclnodejs from '../index.js'` — external consumers are
  unaffected.

> Note: the native addon (`.node`) and `bindings`/prebuild loader stay CJS (wrapped via
> `createRequire()`); generated message code also stays CJS (`generated/`, served through the
> `./generated/*` export). Only the authored JS surface migrates to ESM.

**Re PR #1360 re-use:** it is _Phase 1 only_ (rename `scripts/* → .cjs`) and is already
complete, so "re-using" it means continuing with Phase 2/3 on top of it — there is nothing in
#1360 to re-implement. The remaining dual-package work (Phases 2–4) is a large, atomic
source migration best landed as its own coordinated PR series, not folded into this review.

#### 3.1.1 Post-migration CommonJS/ESM boundary map

After Phases 1–3, the repository is **ESM-first** with a small number of deliberately
isolated CommonJS islands. The diagram below shows where each boundary sits and how it is
crossed.

```
┌──────────────────────────────────────────────────────────────────────────┐
│  package.json  "type": "module"   ← default for every .js in the repo      │
│                "main": ./dist/index.cjs    "module": ./dist/index.js       │
└──────────────────────────────────────────────────────────────────────────┘
        │
        ├─ ESM LAND (the default — almost all source) ──────────────────────┐
        │   index.js, lib/**/*.js, web/**/*.js, rosocket/**/*.js,           │
        │   bin/*.js                                                         │
        │   • import / export, import.meta.url                              │
        │   • export default rcl   (sole public surface)                    │
        │                                                                    │
        │   bridges into CJS via two mechanisms:                            │
        │     (a) static  import x from './foo.cjs'   ← compile-time island │
        │     (b) createRequire(import.meta.url)→require() ← runtime island │
        └────────────────────────────────────────────────────────────────────┘
        │
        ├─ CJS ISLAND #1: static build/generator tooling (.cjs) ────────────┐
        │   rosidl_gen/*.cjs, rostsd_gen/*.cjs, rosidl_gen/templates/*.cjs  │
        │   • imported statically by ESM (import generator from '...cjs')   │
        │   • known paths, resolved at load time                            │
        └────────────────────────────────────────────────────────────────────┘
        │
        ├─ CJS ISLAND #2: generated/ message tree (.js + guard) ────────────┐
        │   generated/**/*.js   +   generated/package.json {"type":"commonjs"}│
        │   • loaded ONLY at runtime via createRequire→require(computedPath) │
        │   • the local package.json flips .js back to CommonJS here         │
        └────────────────────────────────────────────────────────────────────┘
        │
        ├─ CJS ISLAND #3: native + FFI deps ────────────────────────────────┐
        │   third_party/ref-napi, @rclnodejs/ref-struct-di, the .node addon │
        │   • plain npm CommonJS packages, loaded via require()             │
        └────────────────────────────────────────────────────────────────────┘
        │
        ├─ DUAL OUTPUT: dist/ (what consumers actually load) ───────────────┐
        │   tsup emits BOTH for 4 entries → 8 bundles + 8 maps             │
        │   index.js / index.cjs,  web.*,  server.*,  rosocket.*           │
        │   • ESM users → dist/index.js   • CJS users → dist/index.cjs     │
        │   • the .cjs/generated islands are kept EXTERNAL (not bundled)   │
        └────────────────────────────────────────────────────────────────────┘
        │
        └─ STANDALONE SUB-PROJECTS (own package.json, not part of repo land)┐
            demo/electron/*, demo/typescript/*, demo/web/typescript          │
            • consume the PUBLISHED dual package, unaffected by root type    │
            example/**/*.cjs (41 require .default + 3 island)                │
            example/**/*.mjs (5 ESM twins)                                   │
            demo/**/*.cjs (3 converted in-repo scripts)                      │
            └──────────────────────────────────────────────────────────────────┘
```

**File-extension → module-system rules in this repo**

| Pattern                                                            | Resolves as                  | Why                                                                   |
| ------------------------------------------------------------------ | ---------------------------- | --------------------------------------------------------------------- |
| `*.mjs`                                                            | **ESM**                      | extension wins, always                                                |
| `*.cjs`                                                            | **CommonJS**                 | extension wins, always                                                |
| `*.js` under repo root                                             | **ESM**                      | nearest `package.json` = root `"type":"module"`                       |
| `generated/**/*.js`                                                | **CommonJS**                 | nearest `package.json` = `generated/package.json` `"type":"commonjs"` |
| `demo/electron/**`, `demo/typescript/**`, `demo/web/typescript/**` | per their own `package.json` | separate sub-projects                                                 |

**The three bridges (every CJS↔ESM crossing is one of these)**

1. **Static `.cjs` island import** — ESM does `import generator from '../rosidl_gen/index.cjs'`.
   Used when the path is known at authoring time (build/generator tooling). Compile-time, no
   wrapper needed.
2. **`createRequire` runtime require** — an ESM file makes
   `const require = createRequire(import.meta.url)` and calls `require(computedPath)`. Used in
   exactly three places: [lib/interface_loader.js](../lib/interface_loader.js) → generated
   message files (path built from the message name); `lib/native_loader.js` → the native
   `.node` addon (path chosen at runtime by platform); and the same files' lazy
   `require('child_process')` calls. This is the only bridge that is **runtime + dynamic-path**,
   and it is why those targets must stay CJS (a synchronous `require` cannot load ESM).
3. **The published dual package** — tsup emits `dist/*.js` (ESM) and `dist/*.cjs` (CJS), and the
   `exports` map's `import`/`require` conditions route each consumer to the matching one. This is
   the boundary that **end users** cross, and it is invisible to them.

**Invariants that keep it coherent**

- Public surface is ESM-first with a single default export (`export default rcl`); everything
  (Node, publishers, errors, `.require()`) hangs off that one object, so CJS consumers via
  `dist/index.cjs` get the same value with no `.default` confusion (a tsup footer collapses it).
- Messages are reached by **method** (`rclnodejs.require('pkg/msg/Type')`), never by module
  specifier — format-neutral and synchronous, which is why the migration did not have to make
  the API async.
- Every CJS island is either statically known (`.cjs`) or guarded (`generated/package.json`); no
  `.js` file is ambiguous.
- Islands stay **external** in the bundle (tsup's `externalizeIslands` plugin), so runtime
  `require()` paths still resolve to the real on-disk CJS files, not bundled copies.

### 3.2 Async model: make Promises the primary surface

**Current state:** Mixed. Services expose both `sendRequest(req, cb)` and the modern
`sendRequestAsync(req, { signal, timeout })` (which nicely uses `AbortSignal.timeout` and
`AbortSignal.any`). Subscriptions and timers are callback-only.

**Recommendation:**

- Treat **Promise/async as the documented default**, callbacks as the lower-level escape
  hatch. Update README/examples to lead with `await`.
- Thread **`AbortSignal`** consistently through every long-lived/cancelable op (it's already
  in `sendRequestAsync`) — e.g. `waitForMessage`, action goals, `Rate.sleep`.
- Where a callback API exists without a Promise twin, add the `*Async` variant for symmetry.

### 3.3 Async iterators — a JS-native differentiator

rclpy's _experimental_ async API is the natural shape for JS, where `Symbol.asyncIterator`
and `for await...of` are standard. Consider opt-in helpers:

```js
// subscription as an async stream
for await (const msg of node.createSubscription(StringMsg, 'topic').stream()) { ... }

// timer ticks
for await (const info of timer.ticks()) { ... }
```

This is purely additive (no breaking change), reads better than callbacks, and integrates
with `AbortSignal` for cancellation. It would also complement — not replace — the existing
RxJS `ObservableSubscription`.

### 3.4 TypeScript: raise the target, consider generating types

**Current state:** Plain JS + hand-written `.d.ts` in `types/`, validated by `tsd`.
`tsconfig.json` uses `target`/`lib` `es2020` while the runtime requires Node 20 (ES2023).

**Recommendations:**

- Bump `target`/`lib` to **`es2022` (or `es2023`)** to match Node 20. ES2022 gives class
  fields, `.at()`, `Error.cause` (already used in `errors.js`), top-level await in ESM, etc.
- Hand-written declarations drift from implementation. Two paths:
  - **Lower effort:** enable `checkJs` + JSDoc types on `lib/*.js` so `tsc` validates the
    implementation against the public types.
  - **Higher effort / better long-term:** author `lib/` in TypeScript and emit `.d.ts`
    automatically. Builds on the ESM/dual-build move in §3.1 (a 3.0 target).
- Add an explicit `"type"` field discussion to `package.json` when ESM lands.

### 3.5 Tooling & DX

Mostly modern already (ESLint flat config, Prettier, husky + lint-staged, nyc, prebuildify,
tsd). Smaller refinements:

- **ESLint coverage:** the flat config applies `js.configs.recommended` to `lib/**` but no
  TypeScript-aware rules to the JS implementation. Adding `checkJs`/JSDoc (above) closes the
  type-safety gap.
- **Test runner:** Mocha is fine and stable. No action needed; only consider `node:test`
  if/when reducing dev-dependency surface becomes a goal.
- **`.at()`, `structuredClone`, `Array.prototype.findLast`, `Object.hasOwn`** (all ES2022+/
  Node 20) can replace some older idioms (`Object.assign`, `hasOwnProperty.call`,
  manual index math) for readability. Low priority, do opportunistically.
- **Prettier `trailingComma: es5`** could move to `all` once ESM/ES2022 is the baseline.

### 3.6 Web runtime (`web/`)

The new `web/` entry (browser client + `web/server` runtime) is a strong strategic move and
overlaps with the rosbridge story (see [RCLNODEJS_WEB_VS_ROSBRIDGE.md](RCLNODEJS_WEB_VS_ROSBRIDGE.md)).
For the browser surface specifically, ESM and modern types matter even more than on the Node
side, because browser consumers always go through a bundler. Prioritize §3.1/§3.4 for the
`web/` package even if the Node core stays CJS a while longer.

---

## 4. Prioritized recommendations

Ordered by value-to-effort. None of these are blocking; the library is healthy today.

| #   | Recommendation                                                                                                | Theme  | Effort | Priority |
| --- | ------------------------------------------------------------------------------------------------------------- | ------ | ------ | -------- |
| 1   | Add a "Coming from rclpy" doc explaining executor/spin/signal divergences                                     | Docs   | Low    | High     |
| 2   | Land the ESM-first + `tsup` dual-build migration (issue #1358 Phases 2–3) so `import` and `require` both work | ESM    | High   | High     |
| 3   | Bump TS `target`/`lib` to es2022/es2023 to match Node 20                                                      | TS     | Low    | High     |
| 4   | Document Promise/async as the default; lead examples with `await`                                             | Async  | Low    | High     |
| 5   | Add async-iterator helpers (`subscription.stream()`, `timer.ticks()`)                                         | Async  | Medium | Medium   |
| 6   | Thread `AbortSignal` through all cancelable ops                                                               | Async  | Medium | Medium   |
| 7   | Enable `checkJs`+JSDoc so `tsc` validates `lib/` against public types                                         | TS     | Medium | Medium   |
| 8   | Prioritize ESM + modern types for the `web/` package                                                          | Web    | Medium | Medium   |
| 9   | Plan a 3.0 with TS-authored sources (`.d.ts` emitted automatically) on top of the 2.x dual build              | ESM/TS | High   | Low      |
| 10  | Opportunistically adopt ES2022 stdlib (`Object.hasOwn`, `.at()`, etc.)                                        | ES     | Low    | Low      |

---

## 5. Non-goals / explicitly out of scope

- Multi-threaded executor and callback groups (incompatible with the JS model).
- ESM-only / dropping CommonJS before a major version.
- Rewriting the native N-API addon — the binding layer is sound.
- Chasing 1:1 rclpy parity for advanced extensibility (`Waitable`, buffer backends) absent
  user demand.

---

## 6. References

- [RCLPY_PARITY_STATUS.md](RCLPY_PARITY_STATUS.md) — per-feature parity checklist.
- [FEATURES.md](FEATURES.md) — current feature inventory.
- [WEB_RUNTIME_ARCH.md](WEB_RUNTIME_ARCH.md), [WEB_RUNTIME_ROADMAP.md](WEB_RUNTIME_ROADMAP.md) — web runtime design.
- [RCLNODEJS_WEB_VS_ROSBRIDGE.md](RCLNODEJS_WEB_VS_ROSBRIDGE.md) — web client vs rosbridge.
- rclpy source: `rclpy/rclpy/` (node, executors, parameters, qos, lifecycle, actions, experimental async).
