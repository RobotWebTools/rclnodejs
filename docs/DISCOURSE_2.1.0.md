# 🚀 rclnodejs 2.1: Native ESM for a Web-Native ROS 2 SDK

**Tags**: `ros2`, `nodejs`, `web`, `esm`

---

Hi all,

**[rclnodejs 2.1.0](https://github.com/RobotWebTools/rclnodejs/releases/tag/2.1.0) is out** — and it's a step toward making rclnodejs a **web-native SDK for ROS 2**: a platform for bringing ROS 2 to the browser and the modern JavaScript ecosystem. With 2.1.0, the whole package is now **native ESM**, so building **web dashboards, teleop UIs, and browser bridges** on ROS 2 feels native to today's web tooling — with the full ROS 2 runtime still underneath whenever you need real nodes.

> **TL;DR** — 2.1.0 makes rclnodejs **native ESM** end-to-end. `import rclnodejs from 'rclnodejs'` just works, the browser SDK (`rclnodejs/web`) brings ROS 2 to the web, and existing `require()` nodes keep running untouched — the full ROS 2 runtime is still underneath.

## ✨ A web-native SDK for ROS 2

- **ROS 2 on the web** — reach ROS 2 from a web page with `rclnodejs/web`: typed APIs over WebSocket, no proxy, no codegen.
- **Native ESM** — first-class `import` and top-level `await`, ready for Vite/esbuild and the modern web toolchain.
- **Full ROS 2 runtime underneath** — real nodes, pub/sub, services, actions, parameters, lifecycle, whenever you need them.
- **Backward compatible** — every existing CommonJS (`require`) project upgrades with zero code changes.

## 📦 One package, both module systems

```js
// Modern Node / browser — ESM
import rclnodejs from 'rclnodejs'; // full ROS 2 node API
import { connect } from 'rclnodejs/web'; // typed browser SDK
```

```js
// Existing CommonJS nodes — unchanged
const rclnodejs = require('rclnodejs');
```

## 🧭 Where this is heading

Each release moves rclnodejs one step closer to a **typed, web-native way into ROS 2**:

- **[2.0.0](https://discourse.openrobotics.org/t/rclnodejs-2-0-0-typed-web-sdk-for-ros-2-ready-for-lyrical/55061)** 🌐 — a typed browser SDK (`rclnodejs/web`) backed by a capability runtime that exposes _only_ what `web.json` declares, with HTTP `call` / `publish` for non-JS clients.
- **2.1.0** ⚡ — native ESM across the whole package, so rclnodejs sits naturally alongside the rest of your web stack.

**The arc:** from _"a Node.js client that happens to run in browsers"_ → _"a typed, allow-listed Web SDK for ROS 2"_ — with the full ROS 2 node API always underneath, whenever you need real ROS 2 nodes.

## 🔧 Try it

```bash
npm i rclnodejs
```

- 📖 SDK guide: [`web/README.md`](https://github.com/RobotWebTools/rclnodejs/blob/develop/web/README.md)
- ⚡ JS demo (no toolchain): [`demo/web/javascript/`](https://github.com/RobotWebTools/rclnodejs/tree/develop/demo/web/javascript)
- 🧩 TS + Vite demo: [`demo/web/typescript/`](https://github.com/RobotWebTools/rclnodejs/tree/develop/demo/web/typescript)

Feedback welcome — especially from anyone wiring ROS 2 into web frontends. 🙌

Cheers,
Minggang
