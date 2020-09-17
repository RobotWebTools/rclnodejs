# rclnodejs - ROS2 Client Library for JavaScript [![npm](https://img.shields.io/npm/v/rclnodejs.svg)](https://www.npmjs.com/package/rclnodejs)[![Coverage Status](https://coveralls.io/repos/github/RobotWebTools/rclnodejs/badge.svg?branch=develop)](https://coveralls.io/github/RobotWebTools/rclnodejs?branch=develop)[![npm](https://img.shields.io/npm/dm/rclnodejs)](https://www.npmjs.com/package/rclnodejs)[![GitHub license](https://img.shields.io/github/license/RobotWebTools/rclnodejs.svg)](https://github.com/RobotWebTools/rclnodejs/blob/develop/LICENSE)[![node](https://img.shields.io/node/v/rclnodejs.svg)](https://nodejs.org/en/download/releases/)[![dependencies Status](https://david-dm.org/RobotWebTools/rclnodejs/status.svg)](https://david-dm.org/RobotWebTools/rclnodejs)[![npm type definitions](https://img.shields.io/npm/types/rclnodejs)](https://www.npmjs.com/package/rclnodejs)[![code style: prettier](https://img.shields.io/badge/code_style-prettier-ff69b4.svg?style=flat-square)](https://github.com/prettier/prettier)

| Branch  |                                                            Linux Build                                                             |                                                                       macOS Build                                                                       |                                                                                Windows Build                                                                                |
| ------- | :--------------------------------------------------------------------------------------------------------------------------------: | :-----------------------------------------------------------------------------------------------------------------------------------------------------: | :-------------------------------------------------------------------------------------------------------------------------------------------------------------------------: |
| develop | [![Build Status](https://travis-ci.org/RobotWebTools/rclnodejs.svg?branch=develop)](https://travis-ci.org/RobotWebTools/rclnodejs) | [![macOS Build Status](https://circleci.com/gh/RobotWebTools/rclnodejs/tree/develop.svg?style=shield)](https://circleci.com/gh/RobotWebTools/rclnodejs) | [![Build status](https://ci.appveyor.com/api/projects/status/upbc7tavdag1aa5e/branch/develop?svg=true)](https://ci.appveyor.com/project/minggangw/rclnodejs/branch/develop) |
| master  | [![Build Status](https://travis-ci.org/RobotWebTools/rclnodejs.svg?branch=master)](https://travis-ci.org/RobotWebTools/rclnodejs)  | [![macOS Build Status](https://circleci.com/gh/RobotWebTools/rclnodejs/tree/master.svg?style=shield)](https://circleci.com/gh/RobotWebTools/rclnodejs)  |  [![Build status](https://ci.appveyor.com/api/projects/status/upbc7tavdag1aa5e/branch/master?svg=true)](https://ci.appveyor.com/project/minggangw/rclnodejs/branch/master)  |

`rclnodejs` is a Node.js client for the Robot Operating System (ROS 2). It provides a simple and easy JavaScript API for ROS 2 programming. TypeScript declarations are included to support use of rclnodejs in TypeScript projects.

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

## Prerequisites

**Node.js**

- [Node.js](https://nodejs.org/en/) version between 8.12 - 12.x.

**ROS 2 SDK**

- See the ROS 2 SDK [Installation Guide](https://index.ros.org/doc/ros2/Installation/) for details.
- **DON'T FORGET TO [SOURCE THE ROS 2 STARTUP FILES](https://index.ros.org/doc/ros2/Tutorials/Configuring-ROS2-Environment/#source-the-setup-files)**

## Install rclnodejs

Install the rclnodejs version that is compatible with your version of ROS 2 (see table below).

Run the following command for the most current version of rclnodejs

```bash
npm i rclnodejs
```

or to install a specific version of rclnodejs use

```bash
npm i rclnodejs@x.y.z
```

|                                                            RCLNODEJS Version                                                            |                                                                         Compatible ROS 2 Release                                                                         |
| :-------------------------------------------------------------------------------------------------------------------------------------: | :----------------------------------------------------------------------------------------------------------------------------------------------------------------------: |
| [0.15.3 (current)](https://www.npmjs.com/package/rclnodejs/v/0.15.3) ([API](http://robotwebtools.org/rclnodejs/docs/0.15.2/index.html)) | [Foxy Fitzroy](https://github.com/ros2/ros2/releases/tag/release-foxy-20200807) / [Eloquent Elusor](https://github.com/ros2/ros2/releases/tag/release-eloquent-20200124) |
|                                [0.10.3](https://github.com/RobotWebTools/rclnodejs/releases/tag/0.10.3)                                 |                                    [Dashing Diademata - Patch 4](https://github.com/ros2/ros2/releases/tag/release-dashing-20191018)                                     |

- **Note:** to install rclnodejs from GitHub: add `"rclnodejs":"RobotWebTools/rclnodejs#<branch>"` to your `package.json` depdendency section.

## API Documentation

The API documentation is generated by `jsdoc`. To create a local copy run `npm run docs`. Alternatively you can use the prebuilt api documentation found in the `docs/` folder or view the [on-line version](http://robotwebtools.org/rclnodejs/docs/index.html) in your browser.

## Using TypeScript

TypeScript declaration files (\*.d.ts) are included to support developers that wish to use rclnodejs in TypeScript projects.

In your node project, in addition to installing the rclnodejs package, you will need to install the TypeScript compiler and node typings.

```
  npm install typescript @types/node -D
```

Your tsconfig.json file should include the following compiler options:

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

Here's a simple example implemented in TypeScript.

```typescript
import * as rclnodejs from 'rclnodejs';
rclnodejs.init().then(() => {
  const node = rclnodejs.createNode('publisher_example_node');
  const publisher = node.createPublisher('std_msgs/msg/String', 'topic');
  publisher.publish(`Hello ROS 2 from rclnodejs`);
  rclnodejs.spin(node);
});
```

The benefits of using TypeScript become evident when working with more complex use-cases. The ROS 2 messages are defined in the `types/interfaces.d.ts` module. This module is updated as part of the `generate_messages` process. Here's a trivial example of working with a String msg.

```typescript
   const msg: rclnodejs.std_msgs.msg.String = {
     data: 'hello ROS2 from rclnodejs'
   }
```

## Build from Scratch

### Get ready for ROS 2

1.Install ROS 2 from binary package.

ROS 2 is a cross-platform system, which covers Linux, macOS and Windows, and the `rclnodejs` module is developed against the [`master`](https://github.com/ros2/ros2/blob/master/ros2.repos) branch of ROS 2. You can download the latest binary packages from [ROS 2 build farm](http://ci.ros2.org/view/packaging/) and follow the instructions of [Linux](https://index.ros.org/doc/ros2/Installation/Linux-Install-Binary/)/[macOS](https://index.ros.org/doc/ros2/Installation/OSX-Install-Binary/)/[Windows](https://index.ros.org/doc/ros2/Installation/Windows-Install-Binary/) to setup the environment (If you want to run your apps on a stable release of ROS 2, e.g. crystal-clemmys, please see the section `Running on Stable Release of ROS 2).

2.Build ROS 2 from scratch.

Alternatively, you can build ROS 2 from scratch. Please select the platform you want to work on, then reference the instructions of [Linux](https://index.ros.org/doc/ros2/Installation/Linux-Development-Setup/)/[macOS](https://index.ros.org/doc/ros2/Installation/OSX-Development-Setup/)/[Windows](https://index.ros.org/doc/ros2/Installation/Windows-Development-Setup/) to build ROS 2 (please build wiht flag `--merge-install`).

### Install `Node.js`

**Notice:**
`rclnodejs` should only be used with node versions between 8.12 - 12.99. The lowest LTS Node.js we used to verify the unit tests is `8.12.0`. And there is a known issue installing rclnodejs with versions of node >= 13.0.

The `Node.js` version we selected is the LTS [`Erbium`](https://nodejs.org/download/release/latest-erbium/) (12.x). You can install it:

- Download from Node.js offical [website](https://nodejs.org/en/), and install it.
- Use the Node Version Manager ([nvm](https://github.com/creationix/nvm)) to install it.

### Get Code

Open a terminal, and input:

```bash
git clone https://github.com/RobotWebTools/rclnodejs.git
```

### Build Module

Before you build the module, you should make sure the ROS2 environments were loaded. You can check if the `AMENT_PREFIX_PATH` environment variable was set:

- For Windows: `echo %AMENT_PREFIX_PATH%` in the command prompt.

- For Linux and macOS: `echo $AMENT_PREFIX_PATH` in the terminal.

If the `AMENT_PREFIX_PATH` is unset, you should load the ROS2 environments:

- For Windows, open the command prompt and run

```bash
  call <path\to\ros2>\install\local_setup.bat
```

- For Linux and macOS, open the terminal and run:

```bash
  source <path/to/ros2>/install/local_setup.bash
```

This `Node.js` module is built by [node-gyp](https://www.npmjs.com/package/node-gyp), all you have to do is just to run the following command:

```javascript
npm install
```

**Windows-specific**: make sure Python 2.x interpreter is first searched in your `PATH` before running the command. You can change it temporarily by:

```bash
  set PATH=<path\to\python 2.x>;%PATH%
```

## Run Unit Tests

The test suite is implemented using the [mocha](https://www.npmjs.com/package/mocha) JavaScript test framework for node.js. Run the unit tests:

```javascript
npm run test
```

**Windows-specific**: the tests requires in a `Microsoft Visual Studio Native Tools command prompt`, and also make sure Python 3.x interpreter is first searched in your `PATH` before running te test. You can change it temporarily by:

```bash
  set PATH=<path\to\python 3.x>;%PATH%
```

## Troubleshooting

### Maximum call stack size exceeded error when running in Jest

When running tests in Jest, you may see an error like this:

```
RangeError: Maximum call stack size exceeded

      at debug (../node_modules/ref/node_modules/debug/src/debug.js:1:1)
      at Object.writePointer [as _writePointer] (../node_modules/ref/lib/ref.js:746:3)
      at Object.writePointer [as _writePointer] (../node_modules/ref/lib/ref.js:747:11)
      at Object.writePointer [as _writePointer] (../node_modules/ref/lib/ref.js:747:11)
      at Object.writePointer [as _writePointer] (../node_modules/ref/lib/ref.js:747:11)
      at Object.writePointer [as _writePointer] (../node_modules/ref/lib/ref.js:747:11)
      at Object.writePointer [as _writePointer] (../node_modules/ref/lib/ref.js:747:11)
      at Object.writePointer [as _writePointer] (../node_modules/ref/lib/ref.js:747:11)
      at Object.writePointer [as _writePointer] (../node_modules/ref/lib/ref.js:747:11)
      at Object.writePointer [as _writePointer] (../node_modules/ref/lib/ref.js:747:11)
```

This is caused by a bug in `ref` which happens when you `require` it multiple times. There is a fix available for `ref` but it's no longer being maintained and the author has not published it.

If it is required to use Jest, a solution would be to fork `ref` and use npm shrinkwrap to installed a patched version.

### Running with ASAN

#### Linux

To run with google's AddressSanitizer tool, build with `-fsanitize=address` flag,

```sh
CXXFLAGS=-fsanitize=address node-gyp build --debug
```

ASAN needs to be loaded at the start of the process, since rclnodejs is a dynamically loaded library, it will not do so by default. To workaround this, run node with `LD_PRELOAD` to force ASAN to be loaded.

```sh
LD_PRELOAD=$(g++ -print-file-name=libasan.so) node node_modules/.bin/mocha test/test-publisher.js
```

Due to v8's garbage collector, there may be false positives in the leak test, to remove them as much as possible, there is a simple helper script to run gc on exit. To use it, the `--expose-gc` flag needs to be set in node, then run mocha with `-r test/gc-on-exit.js` e.g.

```sh
LD_PRELOAD=$(g++ -print-file-name=libasan.so) node --expose-gc node_modules/.bin/mocha -r test/gc-on-exit.js test/test-publisher.js
```

**Note**: Tests that forks the current process like `test-array.js` will not run gc when they exit. They may report many false positive leaks.

ASAN may report leaks in ref-napi and other modules, there is a suppression file you can use to hide them

```sh
LSAN_OPTIONS=suppressions=suppr.txt node --expose-gc node_modules/.bin/mocha -r test/gc-on-exit.js test/test-publisher.js
```

## Get Involved

### Contributing

If you want to contribute code to this project, first you need to fork the
project. The next step is to send a pull request (PR) for review. The PR will be reviewed by the project team members. Once you have gained "Look Good To Me (LGTM)", the project maintainers will merge the PR.

### Contributors

Special thanks to the people who contribute.

- [martins-mozeiko](https://github.com/martins-mozeiko)
- [Teo Koon Peng](https://github.com/koonpeng)
- [Alex Mikhalev](https://github.com/amikhalev)
- [Wayne Parrott](https://github.com/wayneparrott)
- [Matt Richard](https://github.com/mattrichard)

## License

This project abides by [Apache License 2.0](https://github.com/RobotWebTools/rclnodejs/blob/develop/LICENSE).
