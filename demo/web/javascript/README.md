# JavaScript demo (no build tools)

A single static HTML page that talks to a real ROS 2 graph. No
`roslibjs`, no rosbridge, **no bundler** — just `<script type="module">`
and the SDK's ESM file.

## Run it

```bash
source /opt/ros/<distro>/setup.bash
node demo/web/javascript/server.js
# rclnodejs/web : ws://localhost:9000/capability
#               also http://localhost:9001/capability  (call/publish, curl-able)
# Static files  : http://localhost:8080/
```

Open <http://localhost:8080/> in any modern browser.

`server.js` does two things you'd normally split: it exposes ROS 2 to
the browser **and** serves `index.html` (mapping `/sdk/*` to the
in-repo [`web/`](../../../web/) folder so the page can `import` the
SDK from a plain URL).

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

Real projects use the `rclnodejs-web` CLI against an existing ROS 2
graph instead of running `server.js`:

```bash
npx rclnodejs-web web.json
```

The browser code is unchanged; only the URL it points to changes.

## Putting the SDK in your own project

| Approach                        | When                            | How                                                                |
| ------------------------------- | ------------------------------- | ------------------------------------------------------------------ |
| Plain `<script type="module">`  | static page, hackathon          | host `web/index.js` somewhere, `import { connect } from '/path/to/it'` |
| ESM via npm                     | Vite, Next.js, esbuild, webpack | `npm i rclnodejs`, then `import { connect } from 'rclnodejs/web'`  |
| ESM CDN                         | quick prototypes, codepens      | `import { connect } from 'https://esm.sh/rclnodejs/web'`           |

For a typed version, see the [TypeScript demo](../typescript/).
