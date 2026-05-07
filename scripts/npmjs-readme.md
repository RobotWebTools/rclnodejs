# rclnodejs

`rclnodejs` is a Node.js client library for ROS 2 that provides JavaScript and TypeScript APIs for building ROS 2 applications.

Supported ROS 2 distributions include Humble, Jazzy, Kilted, Lyrical, and Rolling.

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

## Documentation and Examples

- API documentation: [robotwebtools.github.io/rclnodejs/docs](https://robotwebtools.github.io/rclnodejs/docs/index.html)
- Tutorials: [tutorials/](https://github.com/RobotWebTools/rclnodejs/tree/develop/tutorials)
- JavaScript examples: [example/](https://github.com/RobotWebTools/rclnodejs/tree/develop/example)
- TypeScript demos: [demo/typescript/](https://github.com/RobotWebTools/rclnodejs/tree/develop/demo/typescript)
- WebSocket bridge demo (rosocket): [demo/rosocket/](https://github.com/RobotWebTools/rclnodejs/tree/develop/demo/rosocket)
- Electron demos: [demo/electron/](https://github.com/RobotWebTools/rclnodejs/tree/develop/demo/electron)
- Companion CLI: [rclnodejs-cli](https://github.com/RobotWebTools/rclnodejs-cli/)

## Message Generation

rclnodejs auto-generates JavaScript message interfaces and TypeScript declarations during `npm install`, so in most projects you do not need to run anything by hand. If you install additional ROS packages **after** rclnodejs was installed, re-run the generator from your project so the new interfaces are picked up:

```bash
npx generate-ros-messages
```

Generated files are written to `<your-project>/node_modules/rclnodejs/generated/`.

## Using rclnodejs with TypeScript

TypeScript declaration files are included in the package. In most projects, configuring your `tsconfig.json` is sufficient:

```jsonc
{
  "compilerOptions": {
    "module": "commonjs",
    "moduleResolution": "node",
    "target": "es2020",
  },
}
```

Then `import * as rclnodejs from 'rclnodejs'` works the same as the JavaScript example at the top of this README.

## rosocket — Browser ↔ ROS 2 bridge

A tiny WebSocket gateway to ROS 2, built into `rclnodejs`. Exposes ROS 2 topics and services as plain WebSocket URLs — a lightweight alternative to the rosbridge + roslibjs stack. Browsers use only built-in `WebSocket` + `JSON`; no JavaScript library required.

```bash
npx rosocket --port 9000 --topic /chatter:std_msgs/msg/String
```

```js
const ws = new WebSocket('ws://host:9000/topic/chatter');
ws.onmessage = (e) => console.log(JSON.parse(e.data).data);
ws.onopen    = () => ws.send(JSON.stringify({ data: 'hi' }));
```

See the [rosocket guide](https://github.com/RobotWebTools/rclnodejs/tree/develop/rosocket) for the URL scheme, service calls, and the programmatic `startRosocket()` API.

## License

Apache License 2.0
