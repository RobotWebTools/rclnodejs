# rclnodejs [![Linux](https://github.com/RobotWebTools/rclnodejs/actions/workflows/linux-x64-push-test.yml/badge.svg?branch=develop)](https://github.com/RobotWebTools/rclnodejs/actions/workflows/linux-x64-push-test.yml?query=branch%3Adevelop)[![Linux](https://github.com/RobotWebTools/rclnodejs/actions/workflows/linux-arm64-push-test.yml/badge.svg?branch=develop)](https://github.com/RobotWebTools/rclnodejs/actions/workflows/linux-arm64-push-test.yml?query=branch%3Adevelop)

`rclnodejs` is a Node.js client for the Robot Operating System (ROS 2). It provides a simple and easy JavaScript API for ROS 2 programming. TypeScript declarations are included to support use of rclnodejs in TypeScript projects.

\* rclnodejs development and maintenance is limited to all active ROS 2 LTS releases and the Rolling development branch

Here's an example for how to create a ROS 2 node that publishes a string message in a few lines of JavaScript.

```JavaScript
const rclnodejs = require('rclnodejs');
rclnodejs.init().then(() => {
  const node = rclnodejs.createNode('publisher_example_node');
  const publisher = node.createPublisher('std_msgs/msg/String', 'topic');
  publisher.publish(`Hello ROS 2 from rclnodejs`);
  rclnodejs.spin(node);
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

- **Note:** to install rclnodejs from GitHub: add `"rclnodejs":"RobotWebTools/rclnodejs#<branch>"` to your `package.json` dependency section.

## Documentation

API [documentation](https://robotwebtools.github.io/rclnodejs/docs/index.html) is available online.

## JavaScript Examples

Try the [examples](https://github.com/RobotWebTools/rclnodejs/tree/develop/example) to get started.

## Using rclnodejs with TypeScript

TypeScript declaration files are included in the `types/` folder. Configure your `tsconfig.json`:

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
  const node = rclnodejs.createNode('publisher_example_node');
  const publisher = node.createPublisher('std_msgs/msg/String', 'topic');
  publisher.publish(`Hello ROS 2 from rclnodejs`);
  rclnodejs.spin(node);
});
```

See [TypeScript demos](https://github.com/RobotWebTools/rclnodejs/tree/develop/ts_demo) for more examples.

**Note** that the interface.d.ts file is updated each time the generate_messages.js script is run.

## Electron-based Visualization

Create rich, interactive desktop applications using Electron and web technologies like Three.js. Build 3D visualizations, monitoring dashboards, and control interfaces that run on Windows, macOS, and Linux.

Try the `electron_demo/turtle_tf2` demo for real-time coordinate frame visualization with dynamic transforms and keyboard-controlled turtle movement. More examples in [electron_demo](https://github.com/RobotWebTools/rclnodejs/tree/develop/electron_demo).

![demo screenshot](https://github.com/RobotWebTools/rclnodejs/blob/develop/electron_demo/turtle_tf2/turtle-tf2-demo.gif?raw=true)

## License

Apache License Version 2.0
