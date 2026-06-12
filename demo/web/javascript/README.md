# Zero-build ROS 2 in a single HTML page

A single static HTML page that talks to a real ROS 2 graph — just
`<script type="module">` and the SDK's ESM file. No bundler, no
`npm install` for the page itself.

## Run it (two shells)

```bash
cd demo/web/javascript
```

**Shell 1 — runtime + the demo's ROS 2 nodes:**

```bash
source /opt/ros/<distro>/setup.bash
node runtime.mjs
# rclnodejs/web : ws://localhost:9000/capability
#               also http://localhost:9001/capability  (call/publish, curl-able)
#               also http://localhost:9001/capability/subscribe/<name>  (SSE)
```

`runtime.mjs` exposes a tiny `/add_two_ints` service and the shared
`/web_demo_chatter` talker/listener topic (publish from one panel,
receive in the others).

**Shell 2 — static-file server (hosts `index.html` + maps `/sdk/*` to
the in-repo [`web/`](../../../web/) folder so the page can `import`
the SDK from a plain URL):**

```bash
node static.mjs
# Static files : http://localhost:8080/
```

Open <http://localhost:8080/> in any modern browser. Runtime in shell
1, page server in shell 2 — so you can swap in `nginx` / a CDN /
`python3 -m http.server 8080` for shell 2 without touching shell 1.

## What the browser code looks like

```html
<script type="module">
  import { connect } from '/sdk/index.js';
  const ros = await connect('ws://localhost:9000/capability');

  const reply = await ros.call('/add_two_ints', { a: '2n', b: '40n' });
  console.log(reply.sum); // '42n'

  await ros.subscribe('/web_demo_chatter', (msg) => render(msg.data));
  await ros.publish('/web_demo_chatter', { data: 'hi' });
</script>
```

The page also has a **transport toggle** (WebSocket vs. HTTP) so you
can flip the SDK between the two without restarting.

## Same capability, no SDK

Every `call` / `publish` is also reachable as plain HTTP — drive the
runtime from `curl`, Postman, or an AI agent without any JavaScript:

```bash
curl -sS -X POST http://localhost:9001/capability/call/add_two_ints \
  -H 'content-type: application/json' \
  -d '{"a":"7n","b":"35n"}'
# => {"sum":"42n"}
```

This demo's `runtime.mjs` also enables SSE (`new HttpTransport({ sse: true })`),
so `subscribe` is reachable over HTTP as a `text/event-stream` — useful for
clients that can't hold a WebSocket open:

```bash
curl -N http://localhost:9001/capability/subscribe/web_demo_chatter
# event: ready
# data: {"capability":"/web_demo_chatter","subId":"sse"}
#
# event: message
# data: {"data":"hi from curl"}
# …one `message` event per published sample, until you ^C
```

Browser apps should still prefer the WebSocket transport for `subscribe`
(one connection multiplexes every topic). SSE subscribe targets the
curl / AI-agent / server-side persona.

The page also has a **native `EventSource` panel** (section 6) that
subscribes to `/web_demo_chatter` over the same SSE endpoint — no SDK, no
WebSocket, just the browser primitive over plain HTTP. Because the page
(`:8080`) and the HTTP transport (`:9001`) are different origins, the
demo's `runtime.mjs` enables CORS (`new HttpTransport({ sse: true, cors:
true })`) so the cross-origin `EventSource` is allowed. In production,
pass your site's origin instead of `true`.

### Pair it with the stock publisher example

The EventSource panel's topic box defaults to `/web_demo_chatter` (the
shared demo topic), but the runtime also exposes `/topic` so you can
feed the demo from your own node. In a third shell, run the standard
publisher example:

```bash
source /opt/ros/<distro>/setup.bash
node ../../../example/topics/publisher/publisher-example.mjs
# Publishing message: Hello ROS 0
# Publishing message: Hello ROS 1
# …
```

Then set the panel's topic box to `/topic` and click **open
EventSource** — you'll see that node's `Hello ROS N` messages stream in.
The same works over `curl`:

```bash
curl -N http://localhost:9001/capability/subscribe/topic
# event: message
# data: {"data":"Hello ROS 0"}
```

This makes [`publisher-example.mjs`](../../../example/topics/publisher/publisher-example.mjs)
and the web demo a ready-made publisher/subscriber pair for trying the
web runtime against your own publishers.

## Without the bundled `runtime.mjs`

`runtime.mjs` bundles the rclnodejs/web runtime and the demo's sample
ROS 2 nodes (the `/add_two_ints` service) into one process so the demo
runs out of the box. In a real project you already have those ROS 2
nodes running elsewhere, so you only need the runtime. **Replace shell
1's `node runtime.mjs` with the CLI** — shell 2 (`node static.mjs`) and
the browser code are unchanged:

```bash
# shell 1 (instead of `node runtime.mjs`); the `-p rclnodejs` tells npx
# the `rclnodejs-web` binary lives inside the `rclnodejs` package:
npx -p rclnodejs rclnodejs-web web.json

# the publisher / service the demo expects:
ros2 run demo_nodes_cpp add_two_ints_server
# (and a publisher of std_msgs/String on /web_demo_chatter from any source)
```

> Note: this demo enables the SSE subscribe endpoint programmatically in
> `runtime.mjs` via `new HttpTransport({ sse: true, cors: true })`. The
> `rclnodejs-web` CLI can do the same with `--http-sse` and `--http-cors`
> (or `"http": { "sse": true, "cors": "*" }` in `web.json`).
