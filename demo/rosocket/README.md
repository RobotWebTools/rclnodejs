# rosocket demo — ROS 2 in the browser

A minimal end-to-end example of the
[`rosocket`](../../rosocket/README.md) WebSocket gateway. The Node
server runs anywhere ROS 2 is sourced; the HTML page runs in any
modern browser and talks to it over plain `WebSocket` — no client
library required.

## What it shows

- Subscribe to and publish on `/chatter` (`std_msgs/msg/String`).
- Call `/add_two_ints` (`example_interfaces/srv/AddTwoInts`) — the
  service implementation lives in
  [`example/services/service/service-example.mjs`](../../example/services/service/service-example.mjs)
  and is launched in a second terminal.

## Layout

- `server.mjs` — `rclnodejs` node + `startRosocket` bridge only.
- `index.html` — single-file browser client using only built-in
  `WebSocket` and `JSON`.

## Run the bridge

```bash
# 1. Source your ROS 2 distro (humble / jazzy / kilted / lyrical / rolling)
source /opt/ros/$ROS_DISTRO/setup.bash

# 2. Terminal A — start the WebSocket gateway
node demo/rosocket/server.mjs
# [rosocket-demo] listening on ws://localhost:9000 (bind=0.0.0.0)

# 3. Terminal B — start the AddTwoInts service so the browser has
#    something to call
node example/services/service/service-example.mjs
```

The server binds to `0.0.0.0:9000` so it is reachable from any host
that can route to the machine running it.

## Open the page

Open `demo/rosocket/index.html` in any modern browser (double-click,
or `File > Open`). Leave the bridge field as `ws://localhost:9000`
when the browser and the bridge run on the same machine, and click
**Subscribe**, then **Publish**, then **Call**.

If the browser is on a different machine than the bridge (remote ROS
box, container, VM, WSL → host browser, etc.), change the bridge
field to point at the bridge host, e.g. `ws://192.0.2.10:9000` or
`ws://my-ros-host.local:9000`, and reconnect.

> WSL note: WSL2 normally forwards `localhost` to Windows, so
> `ws://localhost:9000` works as-is. On WSL1 or with custom/mirrored
> networking, use the WSL IP (`hostname -I | awk '{print $1}'`)
> instead.

### Verify from the ROS 2 side (optional)

```bash
# Always works (explicit type), even before any browser tab is connected:
ros2 topic echo /chatter std_msgs/msg/String

# Auto-discovery form — only works after a browser tab is connected to
# ws://.../topic/chatter (the bridge creates the subscription on demand)
# and after a publisher exists on the topic (browser Publish, or the
# `ros2 topic pub` command below):
ros2 topic echo /chatter

ros2 topic pub /chatter std_msgs/msg/String "{data: 'hi from ros2'}"
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 7, b: 5}"
```

> If you see `WARNING: topic [/chatter] does not appear to be published yet
> / Could not determine the type for the passed topic`, it just means no
> publisher exists on `/chatter` yet — use the explicit-type form above,
> or click **Subscribe** + **Publish** in the browser first.

Anything you publish from the browser shows up in `ros2 topic echo`,
and any `ros2 topic pub` to `/chatter` shows up in the browser
subscription log.

## URL scheme reminder

```
ws://<host>:<port>/topic/<name>?type=<pkg>/msg/<Type>
ws://<host>:<port>/service/<name>?type=<pkg>/srv/<Type>
```

This demo pre-declares both types server-side via `topicTypes` /
`serviceTypes`, so the browser can omit `?type=`. See
[`rosocket/README.md`](../../rosocket/README.md) for the full
protocol reference.
