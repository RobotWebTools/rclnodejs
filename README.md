# rclnodejs - The ROS 2 Client Library for JavaScript

[![npm](https://img.shields.io/npm/v/rclnodejs)](https://www.npmjs.com/package/rclnodejs)
[![npm downloads](https://img.shields.io/npm/dm/rclnodejs)](https://www.npmjs.com/package/rclnodejs)
[![GitHub stars](https://img.shields.io/github/stars/RobotWebTools/rclnodejs)](https://github.com/RobotWebTools/rclnodejs/stargazers)
[![Coverage Status](https://coveralls.io/repos/github/RobotWebTools/rclnodejs/badge.svg?branch=develop)](https://coveralls.io/github/RobotWebTools/rclnodejs?branch=develop)
[![node](https://img.shields.io/node/v/rclnodejs)](https://nodejs.org/en/download/releases/)
[![TypeScript](https://img.shields.io/npm/types/rclnodejs)](https://www.npmjs.com/package/rclnodejs)

| **ROS 2 Distro** | **CI Status** |
| :---: | :---: |
| [![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Jazzy%20%7C%20Kilted%20%7C%20Rolling-blue?logo=ros)](https://www.ros.org/) | [![Linux x64](https://github.com/RobotWebTools/rclnodejs/actions/workflows/linux-x64-push-test.yml/badge.svg?branch=develop)](https://github.com/RobotWebTools/rclnodejs/actions/workflows/linux-x64-push-test.yml?query=branch%3Adevelop)<br>[![Linux arm64](https://github.com/RobotWebTools/rclnodejs/actions/workflows/linux-arm64-push-test.yml/badge.svg?branch=develop)](https://github.com/RobotWebTools/rclnodejs/actions/workflows/linux-arm64-push-test.yml?query=branch%3Adevelop)<br>[![Windows](https://github.com/RobotWebTools/rclnodejs/actions/workflows/windows-push-test.yml/badge.svg?branch=develop)](https://github.com/RobotWebTools/rclnodejs/actions/workflows/windows-push-test.yml?query=branch%3Adevelop)<br>[![ASan](https://github.com/RobotWebTools/rclnodejs/actions/workflows/linux-x64-asan-test.yml/badge.svg?branch=develop)](https://github.com/RobotWebTools/rclnodejs/actions/workflows/linux-x64-asan-test.yml?query=branch%3Adevelop) |

**rclnodejs** is a Node.js client library for [ROS 2](https://www.ros.org/) that provides comprehensive JavaScript and TypeScript APIs for developing ROS 2 solutions.

**Key features:** Topics, Services, Actions, Parameters, Lifecycle Nodes, TypeScript support, RxJS Observables, Electron integration, and prebuilt binaries for Linux x64/arm64.

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
  [API Documentation](#api-documentation), [Using TypeScript](#using-rclnodejs-with-typescript), [ROS 2 Interface Message Generation](#ros-2-interface-message-generation)
- Features and examples:
  [rclnodejs-cli](#rclnodejs-cli), [Electron-based Visualization](#electron-based-visualization), [Observable Subscriptions](#observable-subscriptions), [Performance Benchmarks](#performance-benchmarks)
- Project docs:
  [Efficient Usage Tips](./docs/EFFICIENCY.md), [FAQ and Known Issues](./docs/FAQ.md), [Building from Scratch](./docs/BUILDING.md), [Contributing](./docs/CONTRIBUTING.md)

## Installation

Choose the path that matches how you plan to use rclnodejs:

- Install from npm: add rclnodejs to your own application.
- Quick Start: run the examples from this repository checkout.

### Prerequisites

- [Node.js](https://nodejs.org/en/) version >= 20.20.2
- [ROS 2 SDK](https://docs.ros.org/en/jazzy/Installation.html)

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

Use this path only if you need a branch or commit that is not yet published to npm.

GitHub installs normally build from source. The published npm package includes prebuilt binaries for supported Linux targets, but this repository does not track those prebuilt artifacts.

```bash
npm install RobotWebTools/rclnodejs#<branch>
```

Or add `"rclnodejs":"RobotWebTools/rclnodejs#<branch>"` to your `package.json` dependency section.

> **Docker:** For containerized development, see the included [Dockerfile](./Dockerfile) for building and testing with different ROS distributions and Node.js versions.

See the [features](./docs/FEATURES.md) and try the [examples](https://github.com/RobotWebTools/rclnodejs/tree/develop/example) to get started.

### Prebuilt Binaries

rclnodejs ships with prebuilt native binaries for common Linux configurations since `v1.5.2`, eliminating the need for compilation during installation. This applies to supported Linux environments when installing the published npm package.

**Supported Platforms:**

- **Ubuntu 22.04 (Jammy)** - ROS 2 Humble
- **Ubuntu 24.04 (Noble)** - ROS 2 Jazzy, Kilted
- **Architectures:** x64, arm64
- **Node.js:** >= 20.20.2 (N-API compatible)

Installations outside this prebuilt matrix automatically fall back to building from source.

**Force Building from Source:**

If you need to build from source even when a prebuilt binary is available, set the environment variable:

```bash
export RCLNODEJS_FORCE_BUILD=1
npm install rclnodejs
```

## Quick Start

Use these steps if you are working from this repository checkout and want to run one of the included examples.

These steps assume the [installation prerequisites](#prerequisites) are already satisfied and your ROS 2 environment has been sourced.

1. Install the repository dependencies from the project root.

```bash
npm install
```

2. Run a publisher example from this checkout.

```bash
node example/topics/publisher/publisher-example.js
```

You should see messages being published once per second.

If you want to build an application instead of running the repository examples, install rclnodejs into your own project with [Install from npm](#install-from-npm) and start from the sample code near the top of this README.

Explore more runnable examples in [example/](https://github.com/RobotWebTools/rclnodejs/tree/develop/example) and step-by-step guides in [tutorials/](./tutorials/).

## rclnodejs-cli

[rclnodejs-cli](https://github.com/RobotWebTools/rclnodejs-cli/) is a separate companion project that provides command-line tooling for working with rclnodejs-based ROS 2 applications. It is particularly useful for creating ROS 2 Node.js packages and working with launch files for multi-node orchestration.

See the rclnodejs-cli repository for installation instructions and the current command set.

## API Documentation

API documentation is available [online](https://robotwebtools.github.io/rclnodejs/docs/index.html). To generate it locally from this repository checkout, run `npm run docs`.

## Electron-based Visualization

Create rich, interactive desktop applications using Electron and web technologies like Three.js. Demos leverage **Electron Forge** for easy packaging on Windows, macOS, and Linux.

|                       Demo                        |                                                              Description                                                              |                            Screenshot                            |
| :-----------------------------------------------: | :-----------------------------------------------------------------------------------------------------------------------------------: | :--------------------------------------------------------------: |
|  **🐢 [turtle_tf2](./electron_demo/turtle_tf2)**  |  Real-time coordinate frame visualization with turtle control. Features TF2 transforms, keyboard control, and dynamic frame updates.  |  ![turtle_tf2](./electron_demo/turtle_tf2/turtle-tf2-demo.png)   |
| **🦾 [manipulator](./electron_demo/manipulator)** | Interactive two-joint robotic arm simulation. Features 3D joint visualization, manual/automatic control, and visual movement markers. | ![manipulator](./electron_demo/manipulator/manipulator-demo.png) |

Explore more examples in [electron_demo](https://github.com/RobotWebTools/rclnodejs/tree/develop/electron_demo).

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

TypeScript example:

```typescript
import * as rclnodejs from 'rclnodejs';
rclnodejs.init().then(() => {
  const node = new rclnodejs.Node('publisher_example_node');
  const publisher = node.createPublisher('std_msgs/msg/String', 'topic');
  publisher.publish(`Hello ROS 2 from rclnodejs`);
  node.spin();
});
```

See [TypeScript demos](https://github.com/RobotWebTools/rclnodejs/tree/develop/ts_demo) for more examples.

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

## ROS 2 Interface Message Generation

ROS client libraries convert IDL message descriptions into target language source code. rclnodejs provides the `generate-ros-messages` script to generate JavaScript message interface files and TypeScript declarations.

**Example usage:**

```javascript
import * as rclnodejs from 'rclnodejs';
let stringMsgObject = rclnodejs.createMessageObject('std_msgs/msg/String');
stringMsgObject.data = 'hello world';
```

### Running Message Generation

Run the message generation script in your project when new ROS packages are installed:

```bash
npx generate-ros-messages
```

Generated files are located at `<yourproject>/node_modules/rclnodejs/generated/`.

> **Note:** This step is not needed for `rclnodejs > 1.5.0` because bundled interfaces are generated during installation. Rerun this command only after adding new ROS packages to your environment.

### IDL Message Generation

In addition to the standard ROS2 message generation (`.msg`, `.srv`, and `.action`), rclnodejs provides advanced support for generating JavaScript message files directly from IDL (Interface Definition Language) files. This feature is particularly useful when working with custom IDL files or when you need more control over the message generation process.

To generate messages from IDL files, use the `generate-messages-idl` npm script:

```bash
npm run generate-messages-idl
```

## Performance Benchmarks

Benchmark results for 1000 iterations with 1024 KB messages (Ubuntu 24.04 WSL2, i7-1185G7):

| Library                 | Topic (ms) | Service (ms) |
| ----------------------- | ---------: | -----------: |
| **rclcpp** (C++)        |        168 |          627 |
| **rclnodejs** (Node.js) |        744 |          927 |
| **rclpy** (Python)      |      1,618 |       15,380 |

These numbers are workload- and environment-specific. See [benchmark/README.md](./benchmark/README.md) for the full setup and methodology.

## Contributing

Please read the [Contributing Guide](./docs/CONTRIBUTING.md) before making a pull request.

Thanks to all [contributors](CONTRIBUTORS.md)!

## License

This project abides by the [Apache License 2.0](https://github.com/RobotWebTools/rclnodejs/blob/develop/LICENSE).
