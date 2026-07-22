# rclnodejs 2.0.0 — typed Web SDK for ROS 2, ready for Lyrical

**Tags**: `ros2`, `nodejs`, `web`

---

Hi all,

[**rclnodejs**](https://github.com/RobotWebTools/rclnodejs) **2.0.0** is out today, on the heels of the **ROS 2 Lyrical Luth** GA on [**May 22, 2026**](https://github.com/ros2/ros2/releases/tag/release-lyrical-20260522). This release adds **`rclnodejs/web`** — a typed browser SDK plus a small runtime that lets a web page talk to ROS 2 over a single WebSocket, with the same capabilities also reachable over plain HTTP for `curl`, Postman, and AI agents.

**New in 2.0.0:**

- A **typed SDK** (`rclnodejs/web`) for browsers and Node — one ROS 2 type name in, request / reply / message all typed.
- A **capability runtime** (`npx rclnodejs-web`) that only exposes what's declared in `web.json` or on the CLI; anything else is rejected with `code: 'not_exposed'` before any ROS 2 API runs.
- An **HTTP fallback** for `call` and `publish` — every capability is also reachable over plain `POST /capability/<verb>/<name>`, so `curl`, Postman, and AI-agent tool-use work without a hand-written shim.

Carried over from [**2.0.0-beta.0**](https://discourse.openrobotics.org/t/rclnodejs-2-0-0-beta-ros-2-lyrical-beta-and-node-js-26-support/54559) (May 2026): the `rosocket` WebSocket gateway, ROS 2 Lyrical Luth (Ubuntu 26.04) support, and Linux x64 / arm64 **N-API prebuilds** — one artifact built against Node.js 20.20.2 runs unchanged on every Node.js ≥ 20.20.2, including 24.x and 26.x — across the full distro matrix (Humble, Jazzy, Kilted, Lyrical, Rolling).

## Browser side, in 5 lines

```ts
import { connect } from 'rclnodejs/web';

const ros = await connect('ws://robot.local:9000/capability');

const reply = await ros.call<'example_interfaces/srv/AddTwoInts'>(
  '/add_two_ints',
  { a: '2n', b: '40n' }
);
console.log(reply.sum); // '42n', typed as `${number}n`
```

Pass a ROS 2 type name as the generic and the corresponding request, reply, or message shape is typed for you.

## Server side, no JavaScript needed

```bash
source /opt/ros/lyrical/setup.bash
npx -p rclnodejs rclnodejs-web --port 9000 --http-port 9001 \
  --call /add_two_ints=example_interfaces/srv/AddTwoInts
# rclnodejs/web listening on ws://localhost:9000/capability (1 capability)
#                also http://localhost:9001/capability (call/publish only)
```

Or pass a `web.json` config file. Either way, **the browser only reaches what the server has declared** — anything else is rejected with `code: 'not_exposed'` before any ROS 2 API runs. With `--http-port` on, every `call` / `publish` is also reachable from plain HTTP:

```bash
curl -X POST -H 'content-type: application/json' \
  -d '{"a":"2n","b":"40n"}' \
  http://localhost:9001/capability/call/add_two_ints
# => {"sum":"42n"}
```

Subscribe stays on WebSocket (one socket multiplexes N topics).

## Tutorial + demos

- SDK guide: [`web/README.md`](https://github.com/RobotWebTools/rclnodejs/blob/2.0.0/web/README.md)
- JS demo (no toolchain): [`demo/web/javascript/`](https://github.com/RobotWebTools/rclnodejs/tree/2.0.0/demo/web/javascript)
- TS + Vite demo: [`demo/web/typescript/`](https://github.com/RobotWebTools/rclnodejs/tree/2.0.0/demo/web/typescript)

Feedback very welcome — particularly on the TypeScript surface, since the wire protocol locks in this release.

Cheers,
Minggang
