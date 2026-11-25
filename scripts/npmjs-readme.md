# rclnodejs

`rclnodejs` is a Node.js client library for [ROS 2](https://www.ros.org/) with JavaScript and TypeScript APIs. It supports topics, services, actions, parameters, lifecycle nodes, RxJS subscriptions, and Electron and browser apps.

Supported ROS 2 distributions include Humble, Jazzy, Lyrical, and Rolling.

Start with [Writing Your First rclnodejs Nodes](https://github.com/RobotWebTools/rclnodejs/blob/develop/tutorials/writing-your-first-rclnodejs-nodes.md) for setup, topics, and service calls.

```javascript
import rclnodejs from 'rclnodejs';

await rclnodejs.init();
const node = new rclnodejs.Node('publisher_example_node');
const publisher = node.createPublisher('std_msgs/msg/String', 'topic');
publisher.publish(`Hello ROS 2 from rclnodejs`);
node.spin();
```

This example assumes your ROS 2 environment is already sourced.

## Installation

### Prerequisites

- [Node.js](https://nodejs.org/en/) version >= 20.20.2
- [ROS 2 SDK](https://docs.ros.org/en/lyrical/Installation.html)

Before installing or running rclnodejs, source your ROS 2 environment:

```bash
source /opt/ros/<distro>/setup.bash
```

### Install rclnodejs

```bash
npm i rclnodejs
```

To install from GitHub instead of npm, run:

```bash
npm install RobotWebTools/rclnodejs#<branch>
```

GitHub installs need newer Node.js/npm versions than the runtime minimum above; see the [build guide](https://github.com/RobotWebTools/rclnodejs/blob/develop/docs/BUILDING.md).

Prebuilt binaries cover Ubuntu 22.04 (Humble), 24.04 (Jazzy), and 26.04 (Lyrical) on x64 and arm64. Other platforms build from source. Set `RCLNODEJS_FORCE_BUILD=1` to force compilation.

## Documentation and Examples

- API documentation: [robotwebtools.github.io/rclnodejs/docs](https://robotwebtools.github.io/rclnodejs/docs/index.html)
- Tutorials: [tutorials/](https://github.com/RobotWebTools/rclnodejs/tree/develop/tutorials)
- JavaScript examples: [example/](https://github.com/RobotWebTools/rclnodejs/tree/develop/example)
- TypeScript demos: [demo/typescript/](https://github.com/RobotWebTools/rclnodejs/tree/develop/demo/typescript)
- Browser demos: [demo/web/](https://github.com/RobotWebTools/rclnodejs/tree/develop/demo/web) (typed Web SDK) and [demo/rosocket/](https://github.com/RobotWebTools/rclnodejs/tree/develop/demo/rosocket) (WebSocket gateway)
- HTTP/SSE actions: [Fibonacci demo](https://github.com/RobotWebTools/rclnodejs/blob/develop/demo/web/api-playground/javascript/README.md#fibonacci-actions)
- Electron demos: [demo/electron/](https://github.com/RobotWebTools/rclnodejs/tree/develop/demo/electron)
- Companion CLI: [rclnodejs-cli](https://github.com/RobotWebTools/rclnodejs-cli/)

## Message Generation

rclnodejs generates JavaScript bindings and TypeScript declarations during `npm install` for ROS 2 `.msg`, `.srv`, and `.action` interfaces in your sourced environment.

After adding ROS interface packages, source their workspace and run from your project:

```bash
npx generate-ros-messages
```

JavaScript bindings are written to `node_modules/rclnodejs/generated/` and TypeScript declarations to `node_modules/rclnodejs/types/interfaces.d.ts`.

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

Use `import rclnodejs from 'rclnodejs'` as in the JavaScript example above.

## Bring ROS 2 to the Web

Build robot dashboards, control panels, and 3D visualizations in JavaScript or TypeScript. Your browser app connects to a Node.js runtime that handles ROS 2 communication.

### Typed Web SDK

[rclnodejs/web](https://github.com/RobotWebTools/rclnodejs/blob/develop/web/README.md) provides `call`, `publish`, `subscribe`, and `action`, with payload types derived from your generated ROS interfaces.

- Choose which services, topics, and actions to expose in `web.json` or CLI flags. Unlisted operations are rejected before reaching ROS 2.
- Use WebSocket for all four operations, or HTTP/SSE for service calls, publishing, and actions. HTTP endpoints also work with `curl` and Postman.
- Export OpenAPI 3.1 with `rclnodejs-web openapi` for API explorers and client generation.

With the [runtime](https://github.com/RobotWebTools/rclnodejs/blob/develop/web/README.md#server) and a ROS `/add_two_ints` service running:

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

SDK subscriptions and action cancellation use WebSocket; cancellation also requires server acceptance.

For browser subscriptions without an SDK, expose `/chatter` and enable HTTP with `--http-port 9001 --http-sse`. Set `--http-cors` to the frontend origin for cross-origin requests:

```js
const es = new EventSource(
  'http://localhost:9001/capability/subscribe/chatter'
);
es.onmessage = (event) => console.log(JSON.parse(event.data));
```

Try [Fibonacci actions in the browser](https://github.com/RobotWebTools/rclnodejs/blob/develop/demo/web/api-playground/javascript/README.md#fibonacci-actions), or watch a [terminal-controlled manipulator in 3D](https://github.com/RobotWebTools/rclnodejs/tree/develop/demo/web/manipulator). See all [web demos](https://github.com/RobotWebTools/rclnodejs/tree/develop/demo/web) for setup and runnable code.

### WebSocket URLs Without an SDK

[rosocket](https://github.com/RobotWebTools/rclnodejs/blob/develop/rosocket/README.md) exposes one topic or service per WebSocket connection. Browser clients use only the built-in `WebSocket` and `JSON` APIs.

```bash
npx rosocket --host 127.0.0.1 --port 9000 --topic /chatter:std_msgs/msg/String
```

## License

Apache License 2.0
