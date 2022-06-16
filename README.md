# rclnodejs - ROS2 Client Library for JavaScript [![npm](https://img.shields.io/npm/v/rclnodejs.svg)](https://www.npmjs.com/package/rclnodejs)[![Coverage Status](https://coveralls.io/repos/github/RobotWebTools/rclnodejs/badge.svg?branch=develop)](https://coveralls.io/github/RobotWebTools/rclnodejs?branch=develop)[![npm](https://img.shields.io/npm/dm/rclnodejs)](https://www.npmjs.com/package/rclnodejs)[![GitHub license](https://img.shields.io/github/license/RobotWebTools/rclnodejs.svg)](https://github.com/RobotWebTools/rclnodejs/blob/develop/LICENSE)[![node](https://img.shields.io/node/v/rclnodejs.svg)](https://nodejs.org/en/download/releases/)[![npm type definitions](https://img.shields.io/npm/types/rclnodejs)](https://www.npmjs.com/package/rclnodejs)[![code style: prettier](https://img.shields.io/badge/code_style-prettier-ff69b4.svg?style=flat-square)](https://github.com/prettier/prettier)

|      **BRANCH**     | **Linux** | **Windows** |
|:-------------------:|:------------:|:---------------:|
|       develop       |  ![GitHub Workflow Status](https://img.shields.io/github/workflow/status/RobotWebTools/rclnodejs/rclnodejs%20-%20Linux%20Build%20and%20Test)  | ![GitHub Workflow Status](https://img.shields.io/github/workflow/status/RobotWebTools/rclnodejs/rclnodejs%20-%20Windows%20Build%20&%20Test)  |
|   humble-hawksbill  | ![GitHub Workflow Status](https://img.shields.io/github/workflow/status/RobotWebTools/rclnodejs/rclnodejs%20-%20Linux%20Build%20and%20Test/humble-hawksbill)  |  ![GitHub Workflow Status (branch)](https://img.shields.io/github/workflow/status/RobotWebTools/rclnodejs/rclnodejs%20-%20Windows%20Build%20&%20Test/humble-hawksbill)  |
| galactic-geochelone  | ![GitHub Workflow Status](https://img.shields.io/github/workflow/status/RobotWebTools/rclnodejs/rclnodejs%20-%20Linux%20Build%20and%20Test/galactic-geochelone)  |  ![GitHub Workflow Status (branch)](https://img.shields.io/github/workflow/status/RobotWebTools/rclnodejs/rclnodejs%20-%20Windows%20Build%20&%20Test/galactic-geochelone)  |
|     foxy-fitzroy  | ![GitHub Workflow Status](https://img.shields.io/github/workflow/status/RobotWebTools/rclnodejs/rclnodejs%20-%20Linux%20Build%20and%20Test/foxy-fitzroy)  |  ![GitHub Workflow Status (branch)](https://img.shields.io/github/workflow/status/RobotWebTools/rclnodejs/rclnodejs%20-%20Windows%20Build%20&%20Test/foxy-fitzroy)  |

**rclnodejs** is a Node.js client library for the Robot Operating System
([ROS 2](https://index.ros.org/doc/ros2/)). It provides a JavaScript API
and tooling for ROS 2 programming. TypeScript declarations, i.e., (\*.d.ts),
are included to support use in TypeScript projects.

Here's an example for how to create a ROS 2 node that publishes a string message in a few lines of JavaScript.

```JavaScript
const rclnodejs = require('rclnodejs');
rclnodejs.init().then(() => {
  const node = new rclnodejs.Node('publisher_example_node');
  const publisher = node.createPublisher('std_msgs/msg/String', 'topic');
  publisher.publish(`Hello ROS 2 from rclnodejs`);
  node.spin();
});
```

## Documentation

- [Installation](#installation)
- [rclnodejs-cli](#rclnodejs-cli)
- [API Documentation](#api-documentation)
- [Using TypeScript](#using-typescript)
- [Examples](https://github.com/RobotWebTools/rclnodejs/tree/develop/example)
- [Efficient Usage Tips](./docs/EFFICIENCY.md)
- [FAQ and Known Issues](./docs/FAQ.md)
- [Building from Scratch](./docs/BUILDING.md)
- [Contributing](./docs/CONTRIBUTING.md)

## Installation

### Prerequisites

Before installing `rclnodejs` please ensure the following software is installed and configured on your system:

- [Nodejs](https://nodejs.org/en/) version between 10.23.1 - 17.x.

- [ROS 2 SDK](https://index.ros.org/doc/ros2/Installation/) for details.
  **DON'T FORGET TO [SOURCE THE ROS 2 SETUP FILE](https://index.ros.org/doc/ros2/Tutorials/Configuring-ROS2-Environment/#source-the-setup-files)**

### Installing rclnodejs

Install the rclnodejs version that is compatible with your version of ROS 2 (see table below).

For the most current version of rclnodejs run:

```bash
npm i rclnodejs
```

To install a specific version of rclnodejs use:

```bash
npm i rclnodejs@x.y.z
```

|                                                            RCLNODEJS Version                                                            |                                                                                                                                                                     Compatible ROS 2 Release                                                                                                                                                                     |
| :-------------------------------------------------------------------------------------------------------------------------------------: | :--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------: |
| [0.21.2 (current)](https://www.npmjs.com/package/rclnodejs/v/0.21.2) ([API](http://robotwebtools.org/rclnodejs/docs/0.21.0/index.html)) | [Humble Hawksbill](https://github.com/ros2/ros2/releases/tag/release-humble-20220523)<br>[Galactic Geochelone](https://github.com/ros2/ros2/releases/tag/release-galactic-20210716)<br>[Foxy Fitzroy](https://github.com/ros2/ros2/releases/tag/release-foxy-20201211)<br>[Eloquent Elusor](https://github.com/ros2/ros2/releases/tag/release-eloquent-20200124) |
|                                [0.10.3](https://github.com/RobotWebTools/rclnodejs/releases/tag/0.10.3)                                 |                                                                                                                                [Dashing Diademata - Patch 4](https://github.com/ros2/ros2/releases/tag/release-dashing-20191018)                                                                                                                                 |

- **Note:** to install rclnodejs from GitHub: add `"rclnodejs":"RobotWebTools/rclnodejs#<branch>"` to your `package.json` depdendency section.

## rclnodejs-cli

[rclnodejs-cli](https://github.com/RobotWebTools/rclnodejs/) is a companion project we recently launched to provide a commandline interface to a set of developer tools for working with this `rclnodejs`. You may find `rclnodejs-cli` particularly useful if you plan to create ROS 2 node(s) and launch files for working with multiple node orchestrations.

```
           _                 _       _
  _ __ ___| |_ __   ___   __| | ___ (_)___
 | '__/ __| | '_ \ / _ \ / _` |/ _ \| / __|
 | | | (__| | | | | (_) | (_| |  __/| \__ \
 |_|  \___|_|_| |_|\___/ \__,_|\___|/ |___/
                                  |__/
Usage: rclnodejs [command] [options]

Options:
  -h, --help                               display help for command

Commands:
  create-package [options] <package-name>  Create a ROS2 package for Nodejs development.
  generate-ros-messages                    Generate JavaScript code from ROS2 IDL interfaces
  help [command]                           display help for command
```

## API Documentation

API documentation is generated by `jsdoc` and can be viewed in the `docs/` folder or [on-line](http://robotwebtools.org/rclnodejs/docs/index.html). To create a local copy of the documentation run `npm run docs`.

## Using rclnodejs with TypeScript

`rclnodejs` API can be used in TypeScript projects. You can find the TypeScript declaration files (\*.d.ts) in the `types/` folder.

Your `tsconfig.json` file should include the following compiler options:

```jsonc
{
  "compilerOptions": {
    "module": "commonjs",
    "moduleResolution": "node",
    "target": "es6"
    // your additional options here
  }
}
```

Here's a short `rclnodejs` TypeScript example:

```typescript
import * as rclnodejs from 'rclnodejs';
rclnodejs.init().then(() => {
  const node = new rclnodejs.Node('publisher_example_node');
  const publisher = node.createPublisher('std_msgs/msg/String', 'topic');
  publisher.publish(`Hello ROS 2 from rclnodejs`);
  node.spin();
});
```

The benefits of using TypeScript become evident when working with more complex use-cases. ROS messages are defined in the `types/interfaces.d.ts` module. This module is updated as part of the `generate-ros-messages` process described in the next section.

## ROS2 Interface Message Generation (important)

ROS components communicate by sending and receiving messages described
by the interface definition language (IDL). ROS client libraries such as
rclnodejs are responsible for converting these IDL message descriptions
into source code of their target language. For this, rclnodejs provides
the npm binary`generate-ros-messages` script that reads the IDL
message files of a ROS environment and generates corresponding JavaScript
message interface files. Additionally, the tool generates the TypeScript
`interface.d.ts` file containing declarations for each IDL message file.

Learn more about ROS interfaces and IDL [here](https://index.ros.org/doc/ros2/Concepts/About-ROS-Interfaces/).

In the following example rclnodejs loads a generated JavaScript message file corresponding to the ROS `std_msgs/msg/String' definition.

```
import * as rclnodejs from 'rclnodejs';
let stringMsgObject = rclnodejs.createMessageObject('std_msgs/msg/String');
stringMsgObject.data = 'hello world';
```

### Maintaining Generated JavaScript Message Files

Message files are generated as a post-install step of the rclnodejs
installation process. Thereafter, you will need to manually run the
rclnodejs message generation script when new ROS message packages are installed
for which your ROS2-nodejs project has a dependency.

### Running `generate-ros-messages` Utility

To run the `generate-ros-messages` script from your Nodejs package, use the `npx` utility included in your Nodejs installation.

```
npx generate-ros-messages
```

The newly generated JavaScript files can be found at
`<yourproject>/node_modules/rclnodejs/generated/`.

## Contributing

Please read the [Contributing Guide]() before making a pull request.

Thank you to all the [people](CONTRIBUTORS.md) who already contributed to rclnodejs!

<div>
<a href="https://github.com/wayneparrott">
  <img src="https://github.com/wayneparrott.png" width="30">
</a>
<a href="https://github.com/koonpeng">
  <img src="https://github.com/koonpeng.png" width="30">
</a>
<a href="https://github.com/mattrichard">
  <img src="https://github.com/mattrichard.png" width="30">
</a>
<a href="https://github.com/felixdivo">
  <img src="https://github.com/felixdivo.png" width="30">
</a>
<a href="https://github.com/martins-mozeiko">
  <img src="https://github.com/martins-mozeiko.png" width="30">
</a>
<a href="https://github.com/amikhalev">
  <img src="https://github.com/amikhalev.png" width="30">
</a>
<a href="https://github.com/kenny-y">
  <img src="https://github.com/kenny-y.png" width="30">
</a>
<a href="https://github.com/qiuzhong">
  <img src="https://github.com/qiuzhong.png" width="30">
</a>
<a href="https://github.com/minggangw">
  <img src="https://github.com/minggangw.png" width="30">
</a>
<a href="https://github.com/hanyia">
  <img src="https://github.com/hanyia.png" width="30">
</a>
</div>

## License

This project abides by the [Apache License 2.0](https://github.com/RobotWebTools/rclnodejs/blob/develop/LICENSE).
