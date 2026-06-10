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

**Key features:** Topics, Services, Actions, Parameters, Lifecycle Nodes, TypeScript support, RxJS Observables, Electron integration, ROS 2 in the browser (typed Web SDK + thin WebSocket gateway — `rclnodejs/web`, `rosocket`), and prebuilt binaries for Linux x64/arm64.

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
  [Installation](#installation), [Quick Start](#quick-start), [Web SDK guide](./web/README.md), [Tutorials](./tutorials/)
- Reference:
  [API Documentation](https://robotwebtools.github.io/rclnodejs/docs/index.html), [ROS 2 Interface Message Generation](#ros-2-interface-message-generation), [Using TypeScript](#using-rclnodejs-with-typescript)
- Features:
  [ROS 2 in the browser](#ros-2-in-the-browser), [Observable Subscriptions](#observable-subscriptions), [Electron-based Visualization](#electron-based-visualization)
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
node example/topics/publisher/publisher-example.cjs
```

More runnable examples in [example/](https://github.com/RobotWebTools/rclnodejs/tree/develop/example) and step-by-step guides in [tutorials/](./tutorials/).

## ROS 2 in the browser

`rclnodejs` ships **two** ways to reach ROS 2 from the browser — pick one based on
how much glue you want to write.

- **[`rclnodejs/web`](./web/README.md)** — **typed, allow-listed,
  curl-able** ROS 2 in the browser. A `web.json` file is your public API;
  the browser SDK types `call` / `publish` / `subscribe` end-to-end
  from your ROS 2 message types; and every capability
  is also a plain HTTP endpoint —
  `curl -X POST http://<host>/capability/call/<name>` — so shell
  scripts, Postman, and AI-agent tool-use just work.
  _New in `2.0.0-beta.0`._

  ```ts
  import { connect } from 'rclnodejs/web';
  const ros = await connect('ws://host:9000/capability');
  const reply = await ros.call<'example_interfaces/srv/AddTwoInts'>(
    '/add_two_ints', { a: '2n', b: '40n' }
  ); // reply.sum is typed as `${number}n`
  ```

- **[`rosocket`](./rosocket/README.md)** — thin WebSocket gateway,
  zero browser dependencies (just built-in `WebSocket` + `JSON`).
  Best for quick prototypes and `roslibjs`-style apps.
  _New in `2.0.0-beta.0`._

  ```bash
  npx rosocket --port 9000 --topic /chatter:std_msgs/msg/String
  ```

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

Build desktop ROS 2 apps with Electron + Three.js, packaged for Windows/macOS/Linux via **Electron Forge**. Featured demo: 🦾 **[manipulator](./demo/electron/manipulator)** — a two-joint arm with manual/automatic control. More in [demo/electron](./demo/electron/).

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

For custom `.idl` files (Interface Definition Language), this repo also exposes `npm run generate-messages-idl`. See [docs/BUILDING.md](./docs/BUILDING.md) for when you'd need it.

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

## More

- **Performance** — faster than `rclpy` and competitive with `rclcpp` for both topic and service round-trips. Full benchmarks in [benchmark/README.md](./benchmark/README.md).
- **Companion CLI** — [`rclnodejs-cli`](https://github.com/RobotWebTools/rclnodejs-cli/) scaffolds rclnodejs application skeletons and orchestrates launch files for multi-node setups.

## Contributing

Please read the [Contributing Guide](./docs/CONTRIBUTING.md) before making a pull request.

Thanks to all [contributors](CONTRIBUTORS.md)!

## License

This project abides by the [Apache License 2.0](https://github.com/RobotWebTools/rclnodejs/blob/develop/LICENSE).
