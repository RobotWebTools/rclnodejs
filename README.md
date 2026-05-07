# rclnodejs - The ROS 2 Client Library for JavaScript

[![npm](https://img.shields.io/npm/v/rclnodejs)](https://www.npmjs.com/package/rclnodejs)
[![npm downloads](https://img.shields.io/npm/dm/rclnodejs)](https://www.npmjs.com/package/rclnodejs)
[![GitHub stars](https://img.shields.io/github/stars/RobotWebTools/rclnodejs)](https://github.com/RobotWebTools/rclnodejs/stargazers)
[![Coverage Status](https://coveralls.io/repos/github/RobotWebTools/rclnodejs/badge.svg?branch=develop)](https://coveralls.io/github/RobotWebTools/rclnodejs?branch=develop)
[![node](https://img.shields.io/node/v/rclnodejs)](https://nodejs.org/en/download/releases/)
[![TypeScript](https://img.shields.io/npm/types/rclnodejs)](https://www.npmjs.com/package/rclnodejs)

| **ROS 2 Distro** | **CI Status** |
| :---: | :---: |
| [![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Jazzy%20%7C%20Kilted%20%7C%20Lyrical%20%7C%20Rolling-blue?logo=ros)](https://www.ros.org/) | [![Linux x64](https://github.com/RobotWebTools/rclnodejs/actions/workflows/linux-x64-push-test.yml/badge.svg?branch=develop)](https://github.com/RobotWebTools/rclnodejs/actions/workflows/linux-x64-push-test.yml?query=branch%3Adevelop)<br>[![Linux arm64](https://github.com/RobotWebTools/rclnodejs/actions/workflows/linux-arm64-push-test.yml/badge.svg?branch=develop)](https://github.com/RobotWebTools/rclnodejs/actions/workflows/linux-arm64-push-test.yml?query=branch%3Adevelop)<br>[![Windows](https://github.com/RobotWebTools/rclnodejs/actions/workflows/windows-push-test.yml/badge.svg?branch=develop)](https://github.com/RobotWebTools/rclnodejs/actions/workflows/windows-push-test.yml?query=branch%3Adevelop)<br>[![ASan](https://github.com/RobotWebTools/rclnodejs/actions/workflows/linux-x64-asan-test.yml/badge.svg?branch=develop)](https://github.com/RobotWebTools/rclnodejs/actions/workflows/linux-x64-asan-test.yml?query=branch%3Adevelop) |

**rclnodejs** is a Node.js client library for [ROS 2](https://www.ros.org/) that provides comprehensive JavaScript and TypeScript APIs for developing ROS 2 solutions.

**Key features:** Topics, Services, Actions, Parameters, Lifecycle Nodes, TypeScript support, RxJS Observables, Electron integration, browser ↔ ROS 2 WebSocket bridge (rosocket), and prebuilt binaries for Linux x64/arm64.

```javascript
const rclnodejs = require('rclnodejs');
rclnodejs.init().then(() => {
  const node = new rclnodejs.Node('publisher_example_node');
  const publisher = node.createPublisher('std_msgs/msg/String', 'topic');
  publisher.publish(`Hello ROS 2 from rclnodejs`);
  node.spin();
});
```

This example assumes your ROS 2 environment is already sourced.

## Documentation

- Get started:
  [Installation](#installation), [Quick Start](#quick-start), [Tutorials](./tutorials/)
- Reference:
  [API Documentation](https://robotwebtools.github.io/rclnodejs/docs/index.html), [Using TypeScript](#using-rclnodejs-with-typescript), [ROS 2 Interface Message Generation](#ros-2-interface-message-generation)
- Features and examples:
  [rosocket](#rosocket--browser--ros-2-bridge), [Observable Subscriptions](#observable-subscriptions), [Electron-based Visualization](#electron-based-visualization), [Performance Benchmarks](#performance-benchmarks), [rclnodejs-cli](#rclnodejs-cli)
- Project docs:
  [Efficient Usage Tips](./docs/EFFICIENCY.md), [FAQ and Known Issues](./docs/FAQ.md), [Building from Scratch](./docs/BUILDING.md), [Contributing](./docs/CONTRIBUTING.md)

## Installation

Most users only need [Install from npm](#install-from-npm) below. If you have cloned this repository and want to run the bundled examples, see [Quick Start](#quick-start) instead.

### Prerequisites

- [Node.js](https://nodejs.org/en/) version >= 20.20.2
- [ROS 2 SDK](https://docs.ros.org/en/lyrical/Installation.html)

Before installing, building, or running rclnodejs, source your ROS 2 environment:

```bash
source /opt/ros/<distro>/setup.bash
```

### Install from npm

Use this path if you want to depend on rclnodejs from your own ROS 2 Node.js application.

```bash
npm i rclnodejs
```

After installation, use the example at the top of this README as a minimal publisher, or continue with [Quick Start](#quick-start) to run the examples in this repository.

### Install from GitHub

Use this path only if you need a branch or commit not yet published to npm. GitHub installs build from source.

```bash
npm install RobotWebTools/rclnodejs#<branch>
```

> **Docker:** For containerized development, see the included [Dockerfile](./Dockerfile) for building and testing with different ROS distributions and Node.js versions.

See the [features](./docs/FEATURES.md) and try the [examples](https://github.com/RobotWebTools/rclnodejs/tree/develop/example) to get started.

### Prebuilt Binaries

rclnodejs ships with prebuilt native binaries for common Linux configurations, so most installs skip compilation.

**Supported Platforms:**

- **Ubuntu 22.04 (Jammy)** - ROS 2 Humble
- **Ubuntu 24.04 (Noble)** - ROS 2 Jazzy, Kilted
- **Ubuntu 26.04 (Resolute)** - ROS 2 Lyrical
- **Architectures:** x64, arm64
- **Node.js:** >= 20.20.2 (N-API compatible)

Installations outside this matrix automatically fall back to building from source. To force a source build even when a prebuilt binary is available:

```bash
export RCLNODEJS_FORCE_BUILD=1
npm install rclnodejs
```

## Quick Start

From a clone of this repository, after sourcing your ROS 2 environment:

1. Install the repository dependencies from the project root.

```bash
npm install
```

2. Run a publisher example from this checkout.

```bash
node example/topics/publisher/publisher-example.js
```

More runnable examples in [example/](https://github.com/RobotWebTools/rclnodejs/tree/develop/example) and step-by-step guides in [tutorials/](./tutorials/).

## ROS 2 Interface Message Generation

rclnodejs auto-generates JavaScript bindings and TypeScript declarations for every ROS 2 `.msg`, `.srv`, and `.action` interface available in your sourced ROS 2 environment. This happens during `npm install`, so in most projects you do not need to run anything by hand.

Use the generated types directly:

```javascript
const rclnodejs = require('rclnodejs');
let stringMsgObject = rclnodejs.createMessageObject('std_msgs/msg/String');
stringMsgObject.data = 'hello world';
```

### Re-running message generation

If you install additional ROS packages **after** rclnodejs was installed, re-run the generator from your project so the new interfaces are picked up:

```bash
npx generate-ros-messages
```

Generated files are written to `<your-project>/node_modules/rclnodejs/generated/`.

### IDL Message Generation

In addition to the standard ROS 2 message generation (`.msg`, `.srv`, `.action`), rclnodejs can also generate JavaScript message files directly from IDL (Interface Definition Language) files. This is useful for custom IDL files or when you need finer control over the generation process.

To generate messages from IDL files:

```bash
npm run generate-messages-idl
```

## Using rclnodejs with TypeScript

TypeScript declaration files are included in the package and exposed through the `types` entry in `package.json`. In most projects, configuring your `tsconfig.json` is sufficient:

```jsonc
{
  "compilerOptions": {
    "module": "commonjs",
    "moduleResolution": "node",
    "target": "es2020",
  },
}
```

Then `import * as rclnodejs from 'rclnodejs'` works the same as the JavaScript example at the top of this README. See [TypeScript demos](https://github.com/RobotWebTools/rclnodejs/tree/develop/demo/typescript) for more.

## rosocket — Browser ↔ ROS 2 bridge

> A tiny WebSocket gateway to ROS 2 — built into `rclnodejs`. _New in `2.0.0-beta.0`._

**rosocket** exposes ROS 2 topics/services as plain WebSocket URLs — a
**lightweight** alternative to the rosbridge + roslibjs stack. Zero browser
code, one Node.js process; browsers use only built-in `WebSocket` + `JSON`,
no JavaScript library required.

```bash
npx rosocket --port 9000 --topic /chatter:std_msgs/msg/String
```

```js
const ws = new WebSocket('ws://host:9000/topic/chatter');
ws.onmessage = (e) => console.log(JSON.parse(e.data).data);
ws.onopen    = () => ws.send(JSON.stringify({ data: 'hi' }));
```

See [rosocket/README.md](./rosocket/README.md) for the URL scheme, service calls, and the programmatic `startRosocket()` API.

## Observable Subscriptions

rclnodejs supports [RxJS](https://rxjs.dev/) Observable subscriptions for reactive programming with ROS 2 messages. Use operators like `throttleTime()`, `debounceTime()`, `map()`, and `combineLatest()` to build declarative message processing pipelines.

```javascript
const { throttleTime, map } = require('rxjs');

const obsSub = node.createObservableSubscription(
  'sensor_msgs/msg/LaserScan',
  '/scan'
);
obsSub.observable
  .pipe(
    throttleTime(200),
    map((msg) => msg.ranges)
  )
  .subscribe((ranges) => console.log('Ranges:', ranges.length));
```

See the [Observable Subscriptions Tutorial](./tutorials/observable-subscriptions.md) for more details.

## Electron-based Visualization

Build interactive desktop ROS 2 apps with Electron + Three.js, packaged for Windows/macOS/Linux via **Electron Forge**. Featured demo: 🦾 **[manipulator](./demo/electron/manipulator)** — a two-joint arm with manual/automatic control.

<p align="left">
  <a href="./demo/electron/manipulator"><img src="./demo/electron/manipulator/manipulator-demo.png" alt="manipulator demo" width="320"></a>
</p>

More in [demo/electron](https://github.com/RobotWebTools/rclnodejs/tree/develop/demo/electron).

## Performance Benchmarks

Benchmark results for 1000 iterations with 1024 KB messages (Ubuntu 24.04 WSL2, i7-1185G7):

| Library                 | Topic (ms) | Service (ms) |
| ----------------------- | ---------: | -----------: |
| **rclcpp** (C++)        |        168 |          627 |
| **rclnodejs** (Node.js) |        744 |          927 |
| **rclpy** (Python)      |      1,618 |       15,380 |

See [benchmark/README.md](./benchmark/README.md) for the full setup and methodology.

## rclnodejs-cli

[rclnodejs-cli](https://github.com/RobotWebTools/rclnodejs-cli/) is a companion project providing command-line tooling for scaffolding rclnodejs application skeletons and working with launch files for multi-node orchestration.

## Contributing

Please read the [Contributing Guide](./docs/CONTRIBUTING.md) before making a pull request.

Thanks to all [contributors](CONTRIBUTORS.md)!

## License

This project abides by the [Apache License 2.0](https://github.com/RobotWebTools/rclnodejs/blob/develop/LICENSE).
