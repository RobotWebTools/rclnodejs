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

[![Nightly Compatibility](https://github.com/RobotWebTools/rclnodejs/actions/workflows/nightly-compatibility.yml/badge.svg?branch=develop&event=schedule)](https://github.com/RobotWebTools/rclnodejs/actions/workflows/nightly-compatibility.yml?query=branch%3Adevelop+event%3Aschedule)

*Nightly Compatibility checks: ROS Rolling [nightlies](https://github.com/ros2/ros2/releases/tag/release-rolling-nightlies) with Node.js 27 [alpha](https://nodejs.org/en/about/previous-releases). See the [build documentation](./docs/BUILDING.md#nightly-compatibility-checks).*

**rclnodejs** is a Node.js client library for [ROS 2](https://www.ros.org/) with JavaScript and TypeScript APIs. It supports topics, services, actions, parameters, lifecycle nodes, RxJS subscriptions, and Electron and browser apps.

Start with [Writing Your First rclnodejs Nodes](./tutorials/writing-your-first-rclnodejs-nodes.md) for setup, topics, and service calls.

```javascript
import rclnodejs from 'rclnodejs';

await rclnodejs.init();
const node = new rclnodejs.Node('publisher_example_node');
const publisher = node.createPublisher('std_msgs/msg/String', 'topic');
publisher.publish(`Hello ROS 2 from rclnodejs`);
node.spin();
```

## Documentation

- Reference: [API Documentation](https://robotwebtools.github.io/rclnodejs/docs/index.html), [Web SDK guide](./web/README.md), [Tutorials](./tutorials/)
- Project docs: [Efficient Usage Tips](./docs/EFFICIENCY.md), [FAQ and Known Issues](./docs/FAQ.md), [Building from Scratch](./docs/BUILDING.md), [Contributing](./docs/CONTRIBUTING.md)

## Installation

### Prerequisites

- [Node.js](https://nodejs.org/en/) version >= 20.20.2
- [ROS 2 SDK](https://docs.ros.org/en/lyrical/Installation.html)

Source your ROS 2 environment before installing, building or running rclnodejs:

```bash
source /opt/ros/<distro>/setup.bash
```

### Add rclnodejs to your project

```bash
npm i rclnodejs
```

Install an unpublished branch or commit with `npm install RobotWebTools/rclnodejs#<branch>`. GitHub installs and source checkouts need newer Node.js/npm versions than the runtime minimum; see the [build guide](./docs/BUILDING.md).

Prebuilt binaries cover Ubuntu 22.04 (Humble), 24.04 (Jazzy, Kilted), and 26.04 (Lyrical) on x64 and arm64. Other platforms build from source. Set `RCLNODEJS_FORCE_BUILD=1` to force compilation, or use the [Dockerfile](./Dockerfile) for container development.

### Run the examples from a clone

```bash
npm install
node example/topics/publisher/publisher-example.mjs
```

More in [example/](https://github.com/RobotWebTools/rclnodejs/tree/develop/example) and step-by-step guides in [tutorials/](./tutorials/).

## Bring ROS 2 to the Web

Build robot dashboards, control panels, and 3D visualizations in JavaScript or TypeScript. Your browser app connects to a Node.js runtime that handles ROS 2 communication.

### Typed Web SDK

[rclnodejs/web](./web/README.md) provides `call`, `publish`, `subscribe`, and `action`, with payload types derived from your generated ROS interfaces.

- Choose which services, topics, and actions to expose in `web.json` or CLI flags. Unlisted operations are rejected before reaching ROS 2.
- Use WebSocket for all four operations, or HTTP/SSE for service calls, publishing, and actions. HTTP endpoints also work with `curl` and Postman.
- Export OpenAPI 3.1 with `rclnodejs-web openapi` for API explorers and client generation.

With the [runtime](./web/README.md#server) and a ROS `/add_two_ints` service running:

```ts
import type {} from 'rclnodejs';
import { connect } from 'rclnodejs/web';

const ros = await connect('ws://localhost:9000/capability');
const reply = await ros.call<'example_interfaces/srv/AddTwoInts'>(
  '/add_two_ints',
  { a: '2n', b: '40n' }
);
console.log(reply.sum);
await ros.close();
```

SDK subscriptions and action cancellation use WebSocket; cancellation also requires server acceptance. Raw HTTP topic subscriptions are available with `--http-sse`.

Try [Fibonacci actions in the browser](./demo/web/api-playground/javascript/README.md#fibonacci-actions), or watch a [terminal-controlled manipulator in 3D](./demo/web/manipulator/). See all [web demos](./demo/web/README.md) for setup and runnable code.

### WebSocket URLs Without an SDK

[rosocket](https://github.com/RobotWebTools/rclnodejs/blob/develop/rosocket/README.md) exposes one topic or service per WebSocket connection. Browser clients use only the built-in `WebSocket` and `JSON` APIs.

```bash
npx rosocket --host 127.0.0.1 --port 9000 --topic /chatter:std_msgs/msg/String
```

## Observable Subscriptions

rclnodejs exposes topic subscriptions as [RxJS](https://rxjs.dev/) Observables. See the [Observable Subscriptions Tutorial](./tutorials/observable-subscriptions.md) for a runnable example and subscription cleanup.

## Electron-based Visualization

See the [Electron demos](./demo/electron/) for desktop ROS 2 apps and packaging with Electron Forge. The [manipulator](./demo/electron/manipulator/) uses Electron and Three.js to control a two-joint arm.

## ROS 2 Interface Message Generation

rclnodejs generates JavaScript bindings and TypeScript declarations during `npm install` for ROS 2 `.msg`, `.srv`, and `.action` interfaces in your sourced environment.

After adding ROS interface packages, source their workspace and run from your project:

```bash
npx generate-ros-messages
```

JavaScript bindings are written to `<your-project>/node_modules/rclnodejs/generated/` and TypeScript declarations to `node_modules/rclnodejs/types/interfaces.d.ts`. For custom `.idl` files, this repo also exposes `npm run generate-messages-idl`.

## Using rclnodejs with TypeScript

The package includes TypeScript declarations. Use NodeNext module resolution in `tsconfig.json`:

```jsonc
{
  "compilerOptions": {
    "module": "NodeNext",
    "moduleResolution": "NodeNext",
    "target": "es2022",
  },
}
```

See the [TypeScript demos](./demo/typescript/).

## More

- [Benchmarks](./benchmark/README.md) record topic publishing and service request-response measurements for rclnodejs v1.5.0, rclcpp, and rclpy.
- [rclnodejs-cli](https://github.com/RobotWebTools/rclnodejs-cli/) creates application skeletons and launch files for multi-node setups.

## Contributing

Please read the [Contributing Guide](./docs/CONTRIBUTING.md) before making a pull request.

Thanks to all [contributors](CONTRIBUTORS.md)!

## License

Licensed under [Apache 2.0](https://github.com/RobotWebTools/rclnodejs/blob/develop/LICENSE).
