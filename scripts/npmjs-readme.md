# rclnodejs

`rclnodejs` is a Node.js client library for ROS 2 that provides JavaScript and TypeScript APIs for building ROS 2 applications.

Supported ROS 2 distributions include Humble, Jazzy, Kilted, and Rolling.

```javascript
const rclnodejs = require('rclnodejs');
rclnodejs.init().then(() => {
  const node = new rclnodejs.Node('publisher_example_node');
  const publisher = node.createPublisher('std_msgs/msg/String', 'topic');
  publisher.publish(`Hello ROS 2 from rclnodejs`);
  node.spin();
});
```

## Installation

### Prerequisites

- [Node.js](https://nodejs.org/en/) version >= 16.13.0
- [ROS 2 SDK](https://docs.ros.org/en/jazzy/Installation.html) - **Don't forget to [source the setup file](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#source-the-setup-files)**

### Install rclnodejs

```bash
npm i rclnodejs
```

- **Note:** To install rclnodejs from GitHub, add `"rclnodejs":"RobotWebTools/rclnodejs#<branch>"` to your `package.json` dependencies.

### Prebuilt Binaries

rclnodejs ships with prebuilt native binaries for common Linux configurations since `v1.5.2`, eliminating the need for compilation during installation.

**Supported Platforms:**

- **Ubuntu 22.04 (Jammy)** - ROS 2 Humble
- **Ubuntu 24.04 (Noble)** - ROS 2 Jazzy, Kilted
- **Architectures:** x64, arm64
- **Node.js:** >= 16.20.2 (N-API compatible)

Installations outside this prebuilt matrix automatically fall back to building from source.

**Force Building from Source:**

```bash
export RCLNODEJS_FORCE_BUILD=1
npm install rclnodejs
```

## Documentation and Examples

- API documentation: [robotwebtools.github.io/rclnodejs/docs](https://robotwebtools.github.io/rclnodejs/docs/index.html)
- JavaScript examples: [example/](https://github.com/RobotWebTools/rclnodejs/tree/develop/example)
- TypeScript demos: [ts_demo/](https://github.com/RobotWebTools/rclnodejs/tree/develop/ts_demo)
- Electron demos: [electron_demo/](https://github.com/RobotWebTools/rclnodejs/tree/develop/electron_demo)
- Companion CLI: [rclnodejs-cli](https://github.com/RobotWebTools/rclnodejs-cli/)

## Message Generation

rclnodejs generates JavaScript message interfaces and TypeScript declarations during installation for `rclnodejs > 1.5.0`. If you install additional ROS packages later, rerun:

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

```typescript
import * as rclnodejs from 'rclnodejs';

rclnodejs.init().then(() => {
  const node = new rclnodejs.Node('publisher_example_node');
  const publisher = node.createPublisher('std_msgs/msg/String', 'topic');
  publisher.publish(`Hello ROS 2 from rclnodejs`);
  node.spin();
});
```

## License

Apache License Version 2.0
