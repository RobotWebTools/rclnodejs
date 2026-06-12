# End-to-end typed ROS 2 in a TypeScript app

A **TypeScript** browser entry built with **Vite**, driven by the typed
SDK so request, reply, and message shapes are visible in your IDE.

```ts
const reply = await ros.call<'example_interfaces/srv/AddTwoInts'>(
  '/add_two_ints', { a: '2n', b: '40n' }
);
// reply.sum is typed as `${number}n` — no hand-written types, no codegen.
```

## Run it (two shells)

```bash
cd demo/web/typescript
npm install
```

**Shell 1 — runtime:**

```bash
source /opt/ros/<distro>/setup.bash
npm run server
# rclnodejs/web : ws://localhost:9000/capability
#               also http://localhost:9001/capability
```

`server.ts` runs the runtime *and* a tiny `/add_two_ints` service +
1 Hz `/web_demo_tick` publisher so every panel has live data.

> The HTTP transport here serves `call` / `publish` only; `subscribe`
> uses WebSocket. HTTP `subscribe` over Server-Sent Events is an opt-in
> (`new HttpTransport({ sse: true })`, or `--http-sse` on the CLI) — see
> the [JavaScript demo](../javascript/README.md) for a working SSE +
> `EventSource` example.

**Shell 2 — Vite dev server:**

```bash
npm run dev
# ➜  Local:  http://localhost:8080/
```

## Without the bundled `server.ts`

`npm run server` is a convenience for this demo — it bundles the
runtime **and** a tiny `/add_two_ints` service + `/web_demo_tick`
publisher into one process so the demo works out of the box.

In a real project you already have those ROS 2 nodes running
elsewhere, so you only need the runtime. **Replace shell 1's
`npm run server` with the CLI** — shell 2 (`npm run dev`) and
`src/main.ts` are unchanged:

```bash
# shell 1 (instead of `npm run server`)
npx rclnodejs-web web.json

# the publisher / service the demo expects:
ros2 run demo_nodes_cpp add_two_ints_server
# (and a publisher of std_msgs/String on /web_demo_tick from any source)
```

The browser doesn't know or care which option is running — it only
sees `ws://localhost:9000/capability` either way.

## Other npm scripts

| Command             | What it does                                         |
| ------------------- | ---------------------------------------------------- |
| `npm run typecheck` | `tsc --noEmit` — silent on success                   |
| `npm run build`     | static bundle in `dist/` (~6 kB JS, ~1.5 kB CSS gz)  |
| `npm run preview`   | serve the built `dist/`                              |
