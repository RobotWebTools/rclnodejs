# rclnodejs v2.0.0 (2026-05-25)

First stable release in the 2.x series. Aligned with the **ROS 2 Lyrical Luth** GA (Ubuntu 26.04, distro id `2605`).

Install with `npm install rclnodejs`. The `latest` tag on npm now points to `2.0.0`.

This release rolls up the 2.0.0-beta.0 baseline (Lyrical support, Node.js 20.20.2 minimum, removal of legacy 1.x compatibility code, and the `rosocket` WebSocket gateway) and adds the headline new capability for the 2.x line: **`rclnodejs/web`** — a typed Browser SDK plus a small capability runtime that lets a web page talk to ROS 2 over a single WebSocket, with the same capabilities also reachable over plain HTTP for `curl`, Postman, and AI agents.

2.0.0 also folds in the full 1.9.0 feature set — `ParameterEventHandler`, `waitForMessage`, timer autostart + `TimerInfo`, QoS overriding options, pre/post-set parameter callbacks, `ClockEvent`, Observable subscriptions (RxJS), `MessageIntrospector`, action-client feedback content filters, and the rest. Anything you had on 1.x is still here; upgrades from 1.9.0 are additive aside from the breaking changes called out below.

## 📣 Important Notes

- rclnodejs 2.0.0 supports **ROS 2 Lyrical Luth** (Ubuntu 26.04, distro id `2605`) in addition to Humble, Jazzy, Kilted, and Rolling.
- Minimum Node.js raised to **20.20.2**. Node.js 16.x and 18.x are no longer supported.
- Linux x64 / arm64 ship **N-API prebuilds** — one artifact built against Node.js 20.20.2 runs unchanged on every Node.js ≥ 20.20.2, including 24.x and 26.x. Electron prebuilds target Electron 34.x.
- Internal compatibility shims and unused utilities have been removed ([#1479](https://github.com/RobotWebTools/rclnodejs/pull/1479)); downstream code depending on undocumented `lib/` internals may need updates.
- `rclnodejs/web` and `rosocket` ship side by side as **siblings, not layers** — they share the monorepo but serve different audiences (typed declarative SDK vs. raw resource-style WebSocket gateway).

## ✨ Highlights

### `rclnodejs/web` — typed Browser SDK + capability runtime

- `rclnodejs/web` capability runtime over WebSocket — declarative `runtime.expose({...})` allow-list ([#1509](https://github.com/RobotWebTools/rclnodejs/pull/1509))
- Typed Browser SDK with single-string-generic API over auto-generated `MessagesMap` / `ServicesMap` ([#1511](https://github.com/RobotWebTools/rclnodejs/pull/1511))
- HTTP transport for `call` / `publish` — `POST /capability/call/<name>` returns the JSON reply ([#1512](https://github.com/RobotWebTools/rclnodejs/pull/1512))
- `rclnodejs-web` CLI launcher — declarative `web.json` config (or `--call` / `--publish` / `--subscribe` flags), zero server code ([#1513](https://github.com/RobotWebTools/rclnodejs/pull/1513))
- WebSocket and HTTP transports default to a shared `/capability` path; SDK auto-derives sibling URLs ([#1515](https://github.com/RobotWebTools/rclnodejs/pull/1515))
- SDK guide (`web/README.md`) plus JS-only and TypeScript + Vite demos under `demo/web/` ([#1514](https://github.com/RobotWebTools/rclnodejs/pull/1514))

### Distros, RMW, and runtime

- Add ROS 2 Lyrical Luth support ([#1496](https://github.com/RobotWebTools/rclnodejs/pull/1496))
- Add explicit LYRICAL distro identifier (`2605`) ([#1488](https://github.com/RobotWebTools/rclnodejs/pull/1488))
- Add lightweight WebSocket bridge (`rosocket`) for browser access to ROS 2 ([#1495](https://github.com/RobotWebTools/rclnodejs/pull/1495))
- Add `rmw_zenoh_cpp` as RMW vendor and update distribution metadata ([#1487](https://github.com/RobotWebTools/rclnodejs/pull/1487))
- Treat unrecognized `ROS_DISTRO` as a future distro ([#1485](https://github.com/RobotWebTools/rclnodejs/pull/1485))
- Refresh Electron prebuild target and demo apps ([#1492](https://github.com/RobotWebTools/rclnodejs/pull/1492))

## 🛡️ Bug Fixes and Safeguards

- Fix SIGSEGV on ROS 2 Rolling caused by rosidl sequence ABI change ([#1480](https://github.com/RobotWebTools/rclnodejs/pull/1480))
- Gate C++ standard by ROS distro ([#1477](https://github.com/RobotWebTools/rclnodejs/pull/1477))

## 🔧 Maintenance

- Raise Node.js minimum to 20.20.2 ([#1478](https://github.com/RobotWebTools/rclnodejs/pull/1478))
- Pump to Node 26.x ([#1497](https://github.com/RobotWebTools/rclnodejs/pull/1497))
- Remove compatibility and custom utility code carried over from 1.x ([#1479](https://github.com/RobotWebTools/rclnodejs/pull/1479))
- Consolidate all demos under `demo/` and add rosocket demo ([#1503](https://github.com/RobotWebTools/rclnodejs/pull/1503))
- Refine README and npmjs landing page ([#1508](https://github.com/RobotWebTools/rclnodejs/pull/1508))
- Surface ROS 2 Lyrical and the 2.0.0-beta.0 baseline in docs ([#1507](https://github.com/RobotWebTools/rclnodejs/pull/1507))
- Pump `lint-staged` and `sinon` to latest ([#1505](https://github.com/RobotWebTools/rclnodejs/pull/1505))
- Reorganize badges on README ([#1483](https://github.com/RobotWebTools/rclnodejs/pull/1483))

## 🚀 CI, Packaging, and Tooling

- Run primary CI lanes on Lyrical instead of Jazzy ([#1506](https://github.com/RobotWebTools/rclnodejs/pull/1506))
- Install ROS 2 Lyrical from apt-deb repo instead of dated tarball ([#1516](https://github.com/RobotWebTools/rclnodejs/pull/1516))
- Install `test-msgs` and `mrpt-msgs` uniformly across all ROS 2 distros ([#1518](https://github.com/RobotWebTools/rclnodejs/pull/1518))
- Add Lyrical Luth lane to Linux prebuild workflows ([#1498](https://github.com/RobotWebTools/rclnodejs/pull/1498))
- Migrate Rolling build to Ubuntu 26.04 ([#1493](https://github.com/RobotWebTools/rclnodejs/pull/1493))
- Add GitHub Actions workflow to deploy JSDoc to GitHub Pages ([#1490](https://github.com/RobotWebTools/rclnodejs/pull/1490))
- Use `npm install` in `deploy-docs` workflow ([#1491](https://github.com/RobotWebTools/rclnodejs/pull/1491))
- Trigger `npm-publish` on prerelease tags ([#1501](https://github.com/RobotWebTools/rclnodejs/pull/1501))

## Getting Started with `rclnodejs/web`

```ts
// Browser
import { connect } from 'rclnodejs/web';

const ros = await connect('ws://robot.local:9000/capability');
const reply = await ros.call<'example_interfaces/srv/AddTwoInts'>(
  '/add_two_ints',
  { a: '2n', b: '40n' }
);
console.log(reply.sum); // '42n'
```

```bash
# Server (no JavaScript needed)
source /opt/ros/lyrical/setup.bash
npx -p rclnodejs rclnodejs-web --port 9000 --http-port 9001 \
  --call /add_two_ints=example_interfaces/srv/AddTwoInts
```

See the [SDK guide](https://github.com/RobotWebTools/rclnodejs/blob/2.0.0/web/README.md) and [demos](https://github.com/RobotWebTools/rclnodejs/tree/2.0.0/demo/web).

---

**Full Changelog**: [1.9.0...2.0.0](https://github.com/RobotWebTools/rclnodejs/compare/1.9.0...2.0.0)
