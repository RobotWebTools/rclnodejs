Hi all,

For those new to the project: [**rclnodejs**](https://github.com/RobotWebTools/rclnodejs) is the Node.js client library for ROS 2, maintained under the Robot Web Tools umbrella. It lets you write ROS 2 nodes — publishers, subscribers, services, actions, parameters, lifecycle, etc. — in plain JavaScript or TypeScript, with a native binding to `rcl`/`rmw` so messages stay zero-copy where possible. It’s a good fit for web dashboards, Electron desktop apps, browser bridges, scripting, and rapid prototyping.

I’m happy to announce that [**rclnodejs 2.0.0-beta.0**](https://github.com/RobotWebTools/rclnodejs/releases/tag/2.0.0-beta.0) is out — the first preview of the 2.x line, with first-class support for the upcoming **ROS 2 Lyrical Luth** release on **Ubuntu 26.04** and the latest **Node.js 26**.

## What’s new in 2.0.0-beta.0

- **ROS 2 Lyrical Luth (Ubuntu 26.04)** is supported in addition to existing distros (Humble / Jazzy / Kilted / Rolling).
- **Node.js 26.x** is supported, with Linux x64 and arm64 prebuilds so `npm install` works without a local toolchain.

## New: rosocket — talk to ROS 2 from a browser, no JS library

This release ships a lightweight WebSocket bridge called **rosocket** plus an end-to-end demo. The point: a browser tab can subscribe and publish to topics (and call services) using only the built-in `WebSocket` and `JSON` APIs — no rosbridge or roslibjs needed.

URL convention:

```
ws://<host>:<port>/topic/<name>
ws://<host>:<port>/service/<name>
```

Browser-side pub/sub in a few lines:

```js
const BRIDGE = 'ws://localhost:9000';

// Subscribe — every published message arrives as onmessage.
const sub = new WebSocket(`${BRIDGE}/topic/chatter`);
sub.onmessage = (ev) => console.log('recv:', ev.data); // {"data":"hi"}

// Publish — open, send JSON, close.
const pub = new WebSocket(`${BRIDGE}/topic/chatter`);
pub.onopen = () => {
  pub.send(JSON.stringify({ data: 'hello from browser' }));
  setTimeout(() => pub.close(), 200);
};
```

Live walkthrough:

https://www.youtube.com/watch?v=9F0bQTDxUvY

Code: [demo/rosocket](https://github.com/RobotWebTools/rclnodejs/tree/develop/demo/rosocket).

## Electron desktop visualization demos

For richer UIs, the same library powers cross-platform desktop apps via Electron — HTML/CSS/Three.js/WebGL on the front end, with real ROS 2 nodes running in the Electron main process. The demos cover topics, a car controller, a manipulator, and a turtle TF2 visualizer.

https://www.youtube.com/watch?v=M7DYc2pysdo

Code: [demo/electron](https://github.com/RobotWebTools/rclnodejs/tree/develop/demo/electron).

Cheers,
Minggang
