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
import rclnodejs from 'rclnodejs';

await rclnodejs.init();
const node = new rclnodejs.Node('publisher_example_node');
const publisher = node.createPublisher('std_msgs/msg/String', 'topic');
publisher.publish(`Hello ROS 2 from rclnodejs`);
node.spin();
```

## Documentation

- Reference:
  [API Documentation](https://robotwebtools.github.io/rclnodejs/docs/index.html), [Web SDK guide](./web/README.md), [Tutorials](./tutorials/)
- Project docs:
  [Efficient Usage Tips](./docs/EFFICIENCY.md), [FAQ and Known Issues](./docs/FAQ.md), [Building from Scratch](./docs/BUILDING.md), [Contributing](./docs/CONTRIBUTING.md)

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

For a branch or commit not yet published to npm, use
`npm install RobotWebTools/rclnodejs#<branch>`, which builds from source.

Prebuilt binaries ship for Ubuntu 22.04 (Humble), 24.04 (Jazzy, Kilted) and
26.04 (Lyrical) on x64 and arm64, so most installs skip compilation; anything
else builds from source. Set `RCLNODEJS_FORCE_BUILD=1` to always build from
source, and see the [Dockerfile](./Dockerfile) for containerized development.

### Run the examples from a clone

```bash
npm install
node example/topics/publisher/publisher-example.mjs
```

More in [example/](https://github.com/RobotWebTools/rclnodejs/tree/develop/example) and step-by-step guides in [tutorials/](./tutorials/).

## Bring ROS 2 to the Web

`rclnodejs` ships **two** ways to reach ROS 2 from the browser — pick one based on
how much glue you want to write.

- **[`rclnodejs/web`](./web/README.md)** — a typed layer over your ROS 2 graph:
  you allow-list capabilities in `web.json` or via CLI flags; anything else is
  rejected before it reaches ROS 2. Best for typed web apps and HTTP clients.
  - **Typed SDK** — `call`, `publish` and `subscribe`, typed end-to-end from
    your generated message and service types.
  - **Two transports** — WebSocket by default, and plain HTTP so `call` and
    `publish` work from `curl`, Postman or `fetch()`. `subscribe` needs
    WebSocket, or `--http-sse` to stream it as Server-Sent Events.
  - **OpenAPI 3.1** — `rclnodejs-web openapi` emits a machine-readable spec
    for codegen, API explorers and agent tool-use.

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

  ```bash
  npx rosocket --port 9000 --topic /chatter:std_msgs/msg/String
  ```

## Observable Subscriptions

rclnodejs supports [RxJS](https://rxjs.dev/) Observable subscriptions for reactive programming with ROS 2 messages — operators like `throttleTime()`, `debounceTime()`, `map()`, and `combineLatest()` build declarative message processing pipelines. See the [Observable Subscriptions Tutorial](./tutorials/observable-subscriptions.md) for the full API and runnable examples.

## Electron-based Visualization

Build desktop ROS 2 apps with Electron + Three.js, packaged for Windows/macOS/Linux via **Electron Forge**. Featured demo: 🦾 **[manipulator](./demo/electron/manipulator)** — a two-joint arm with manual/automatic control. More in [demo/electron](./demo/electron/).

## ROS 2 Interface Message Generation

rclnodejs auto-generates JavaScript bindings and TypeScript declarations for every ROS 2 `.msg`, `.srv`, and `.action` interface in your sourced environment. This runs during `npm install`, so in most projects you never invoke it by hand.

If you install additional ROS packages afterwards, re-run it from your project so the new interfaces are picked up:

```bash
npx generate-ros-messages
```

Generated files are written to `<your-project>/node_modules/rclnodejs/generated/`. For custom `.idl` files, this repo also exposes `npm run generate-messages-idl`.

## Using rclnodejs with TypeScript

TypeScript declaration files are included in the package and exposed through the `types` entry in `package.json`. In most projects, configuring your `tsconfig.json` is sufficient:

```jsonc
{
  "compilerOptions": {
    "module": "NodeNext",
    "moduleResolution": "NodeNext",
    "target": "es2022",
  },
}
```

Then `import * as rclnodejs from 'rclnodejs'` works as in the example above. See [TypeScript demos](https://github.com/RobotWebTools/rclnodejs/tree/develop/demo/typescript).

## More

- **Performance** — faster than `rclpy` and competitive with `rclcpp` for both topic and service round-trips. Full benchmarks in [benchmark/README.md](./benchmark/README.md).
- **Companion CLI** — [`rclnodejs-cli`](https://github.com/RobotWebTools/rclnodejs-cli/) scaffolds rclnodejs application skeletons and orchestrates launch files for multi-node setups.

## Contributing

Please read the [Contributing Guide](./docs/CONTRIBUTING.md) before making a pull request.

Thanks to all [contributors](CONTRIBUTORS.md)!

## License

This project abides by the [Apache License 2.0](https://github.com/RobotWebTools/rclnodejs/blob/develop/LICENSE).
