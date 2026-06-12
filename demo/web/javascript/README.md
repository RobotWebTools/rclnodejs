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

Every `call` / `publish` is reachable as plain HTTP — drive the runtime
from `curl`, Postman, or an AI agent, no JavaScript required:

```bash
curl -sS -X POST http://localhost:9001/capability/call/add_two_ints \
  -H 'content-type: application/json' -d '{"a":"7n","b":"35n"}'
# => {"sum":"42n"}
```

The demo also enables SSE (`new HttpTransport({ sse: true })`), so
`subscribe` works over HTTP as a `text/event-stream` — handy for clients
that can't hold a WebSocket open:

```bash
curl -N http://localhost:9001/capability/subscribe/web_demo_chatter
# event: ready
# data: {"capability":"/web_demo_chatter","subId":"sse"}
#
# event: message
# data: {"data":"hi from curl"}
```

The page's **native `EventSource` panel** (section 6) reads this same
stream — no SDK, no WebSocket. It works cross-origin (`:8080` → `:9001`)
because the demo also enables CORS (`new HttpTransport({ sse: true, cors:
true })`); in production, pass your site's origin instead of `true`.

> For browser apps, prefer the WebSocket transport for `subscribe` — one
> connection multiplexes every topic. SSE targets the curl / AI-agent /
> server-side persona.

### Pair it with your own publisher

The runtime also exposes `/topic`, so you can feed the demo from any ROS 2
node instead of the in-page publisher. Run the stock publisher example in
a third shell, then point the EventSource panel (or `curl`) at `/topic`:

```bash
source /opt/ros/<distro>/setup.bash
node ../../../example/topics/publisher/publisher-example.mjs
# Publishing message: Hello ROS 0, 1, 2, …

curl -N http://localhost:9001/capability/subscribe/topic
# event: message
# data: {"data":"Hello ROS 0"}
```

## Without the bundled `runtime.mjs`

`runtime.mjs` bundles the runtime and the demo's sample nodes into one
process so it runs out of the box. In a real project those nodes already
run elsewhere, so you only need the runtime — replace shell 1 with the
CLI (shell 2 and the browser code are unchanged):

```bash
# the `-p rclnodejs` tells npx the binary lives in the rclnodejs package:
npx -p rclnodejs rclnodejs-web web.json

# plus the service the demo expects (and any std_msgs/String publisher
# on /web_demo_chatter):
ros2 run demo_nodes_cpp add_two_ints_server
```

> The bundled `runtime.mjs` enables SSE + CORS via
> `new HttpTransport({ sse: true, cors: true })`. The CLI does the same
> with `--http-sse` / `--http-cors` (or `"http": { "sse": true, "cors":
> "*" }` in `web.json`).
