# ROS 2 in the browser — no build tools required

A single static HTML page that talks to a real ROS 2 graph — just
`<script type="module">` and the SDK's ESM file. No bundler, no
`npm install` for the page itself.

## Run it

```bash
source /opt/ros/<distro>/setup.bash
node demo/web/javascript/server.js
# rclnodejs/web : ws://localhost:9000/capability
#               also http://localhost:9001/capability  (call/publish, curl-able)
# Static files  : http://localhost:8080/
```

Open <http://localhost:8080/> in any modern browser.

`server.js` is a convenience for this demo: it runs the rclnodejs/web
runtime, exposes a tiny `/add_two_ints` service + 1 Hz
`/web_demo_tick` publisher (so every panel has live data), **and**
serves `index.html` (mapping `/sdk/*` to the in-repo
[`web/`](../../../web/) folder so the page can `import` the SDK from
a plain URL).

## What the browser code looks like

```html
<script type="module">
  import { connect } from '/sdk/index.js';
  const ros = await connect('ws://localhost:9000/capability');

  const reply = await ros.call('/add_two_ints', { a: '2n', b: '40n' });
  console.log(reply.sum); // '42n'

  await ros.subscribe('/web_demo_tick', (msg) => render(msg.data));
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

Subscribe stays on WebSocket.

## Without the bundled `server.js`

`server.js` bundles the runtime, the sample ROS 2 nodes, and the
static-file server into one process so the demo runs out of the
box. In a real project you already have those ROS 2 nodes running
elsewhere, and you serve the page from your normal web host.
**Replace `node server.js` with the CLI** — the browser code is
unchanged, only the URL it points to changes:

```bash
# instead of `node server.js` (the `-p rclnodejs` tells npx the
# `rclnodejs-web` binary lives inside the `rclnodejs` package):
npx -p rclnodejs rclnodejs-web web.json

# the publisher / service the demo expects:
ros2 run demo_nodes_cpp add_two_ints_server
# (and a publisher of std_msgs/String on /web_demo_tick from any source)
```

## Putting the SDK in your own project

| Approach                        | When                            | How                                                                |
| ------------------------------- | ------------------------------- | ------------------------------------------------------------------ |
| Plain `<script type="module">`  | static page, hackathon          | host `web/index.js` somewhere, `import { connect } from '/path/to/it'` |
| ESM via npm                     | Vite, Next.js, esbuild, webpack | `npm i rclnodejs`, then `import { connect } from 'rclnodejs/web'`  |
| ESM CDN                         | quick prototypes, codepens      | `import { connect } from 'https://esm.sh/rclnodejs/web'`           |

For a typed version, see the [TypeScript demo](../typescript/).
