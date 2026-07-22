# rclnodejs v2.1.1

Adds Server-Sent Events (SSE) `subscribe` support to the `rclnodejs/web` HTTP transport ([#1537](https://github.com/RobotWebTools/rclnodejs/pull/1537)), so browsers and other HTTP clients can stream live ROS 2 topic data over plain HTTP — no WebSocket, no SDK. Enable it with `sse: true` (or `--http-sse` on the CLI); `cors` is also now supported (`--http-cors <origin>`).

```js
const es = new EventSource('http://localhost:9001/capability/subscribe/scan');
es.addEventListener('message', (e) => console.log(JSON.parse(e.data)));
```

Also in this release: `node-gyp` pinned to `^12.2.0` (`13.x` dropped Node 20 support, breaking the arm64 CI leg), `c8` bumped to `^12.0.0` ([#1560](https://github.com/RobotWebTools/rclnodejs/pull/1560)), and the JSDoc site now rebuilds from git tags instead of gh-pages branch state ([#1553](https://github.com/RobotWebTools/rclnodejs/pull/1553)).

```bash
npm i rclnodejs
```

Full Changelog: [2.1.0...2.1.1](https://github.com/RobotWebTools/rclnodejs/compare/2.1.0...2.1.1)
