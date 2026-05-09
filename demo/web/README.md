# rclnodejs/web — talk to ROS 2 from a browser

> The web side of `rclnodejs`. One JSON file, one shell command, your
> browser talks to ROS 2 over a single typed WebSocket — and the same
> capabilities are reachable from plain HTTP for `curl`, Postman, and
> AI agents.

Two demos, same browser API:

| Path | Pick this if you… | What you write |
|---|---|---|
| **[`./javascript/`](./javascript/)** | want a single static page, no build tools, no `npm install` for the page | a `<script>` tag |
| **[`./typescript/`](./typescript/)** | already have a Vite / Next.js / React / Vue / Svelte project | `import { connect } from 'rclnodejs/web'` |

## You don't have to write any backend code

The bundled `rclnodejs-web` CLI takes a JSON allow-list and stands up
the runtime for you:

```bash
source /opt/ros/<distro>/setup.bash
npx rclnodejs-web web.json
# rclnodejs/web listening on ws://localhost:9000/capability (4 capabilities)
#                also http://localhost:9001/capability (call/publish only)
```

`web.json` lists every topic and service the browser is allowed to
reach:

```json
{
  "port": 9000,
  "http": { "port": 9001 },
  "expose": {
    "call":      { "/add_two_ints":    "example_interfaces/srv/AddTwoInts" },
    "publish":   { "/cmd_vel":         "geometry_msgs/msg/Twist" },
    "subscribe": { "/scan":            "sensor_msgs/msg/LaserScan" }
  }
}
```

A complete example is checked in at
[`javascript/web.json`](./javascript/web.json), and `rclnodejs-web --help`
shows every CLI flag.

## What the browser code looks like

```js
import { connect } from 'rclnodejs/web';

const ros = await connect('ws://localhost:9000/capability');

// Service call
const reply = await ros.call('/add_two_ints', { a: '2n', b: '40n' });
console.log(reply.sum);                       // '42n'

// Subscribe
const sub = await ros.subscribe('/scan', (msg) => render(msg));

// Publish
await ros.publish('/cmd_vel', { linear: { x: 0.5 } });
```

That's the whole API. No connection state machine, no `roslibjs`, no
operations dictionary.

## Things you'll likely ask

- **Will my browser need a ROS install?** No. The browser only speaks
  to the WebSocket (or HTTP) endpoint that `rclnodejs/web` exposes.
- **Can I deploy the page anywhere?** Yes — the page is just static
  HTML/JS/CSS. S3, Vercel, GitHub Pages, nginx, your robot's SD card.
  `rclnodejs/web` runs wherever you have ROS 2.
- **Can the browser reach `/dangerous_topic` if it knows the name?** No
  — anything not listed in `web.json` is rejected with
  `code: 'not_exposed'`. The allow-list is the contract.
- **What about HTTPS / `wss://`?** Put nginx, Caddy, or any TLS proxy
  in front of `rclnodejs-web`. The runtime itself speaks plain `ws://`
  and `http://`.
- **What about auth?** Today: gate at the connection level via the
  `verifyClient(req)` / `verifyRequest(req)` hooks on
  `WebSocketTransport` / `HttpTransport`. Per-capability scopes are on
  the roadmap.
- **Does it support Actions?** Not yet — coming in a follow-up release.
- **64-bit integer fields look weird** (`"42n"`)? — That's how
  JavaScript handles `int64` on the wire. The browser SDK and your
  ROS 2 messages round-trip them transparently.
