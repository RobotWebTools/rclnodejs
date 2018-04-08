# rclnodejs - ROS Client Library for JavaScript language[![Coverage Status](https://coveralls.io/repos/github/RobotWebTools/rclnodejs/badge.svg?branch=develop)](https://coveralls.io/github/RobotWebTools/rclnodejs?branch=develop)[![npm](https://img.shields.io/npm/dt/rclnodejs.svg)](https://www.npmjs.com/package/rclnodejs)[![GitHub license](https://img.shields.io/github/license/RobotWebTools/rclnodejs.svg)](https://github.com/RobotWebTools/rclnodejs/blob/develop/LICENSE)[![node](https://img.shields.io/node/v/rclnodejs.svg)](https://nodejs.org/en/download/releases/)[![dependencies Status](https://david-dm.org/RobotWebTools/rclnodejs/status.svg)](https://david-dm.org/RobotWebTools/rclnodejs)

Branch | Linux Build | macOS Build | Windows Build |
------------ |  :-------------: | :-------------: | :-------------: |
develop | [![Build Status](https://travis-ci.org/RobotWebTools/rclnodejs.svg?branch=develop)](https://travis-ci.org/RobotWebTools/rclnodejs) | [![macOS Build Status](https://circleci.com/gh/RobotWebTools/rclnodejs/tree/develop.svg?style=shield)](https://circleci.com/gh/RobotWebTools/rclnodejs) | [![Build status](https://ci.appveyor.com/api/projects/status/upbc7tavdag1aa5e/branch/develop?svg=true)](https://ci.appveyor.com/project/minggangw/rclnodejs/branch/develop)
master | [![Build Status](https://travis-ci.org/RobotWebTools/rclnodejs.svg?branch=master)](https://travis-ci.org/RobotWebTools/rclnodejs) | [![macOS Build Status](https://circleci.com/gh/RobotWebTools/rclnodejs/tree/master.svg?style=shield)](https://circleci.com/gh/RobotWebTools/rclnodejs) | [![Build status](https://ci.appveyor.com/api/projects/status/upbc7tavdag1aa5e/branch/master?svg=true)](https://ci.appveyor.com/project/minggangw/rclnodejs/branch/master)

[![NPM](https://nodei.co/npm/rclnodejs.png)](https://nodei.co/npm/rclnodejs/)

## Build Environment

### Get ready for ROS 2

1.Build from scratch.

ROS is a cross-platform system, which covers Linux, macOS and Windows, and the `rclnodejs` module is developed on the [`master`](https://github.com/ros2/ros2/blob/master/ros2.repos) branch of ROS, so you have to build ROS from scratch at the present stage. Please select the platform you want to work on, then reference the instruction to build ROS.

* [Linux](https://github.com/ros2/ros2/wiki/Linux-Development-Setup)
* [macOS](https://github.com/ros2/ros2/wiki/OSX-Development-Setup)
* [Windows](https://github.com/ros2/ros2/wiki/Windows-Development-Setup)

2.Get the binary package.

Alternatively, you can download the latest binary package of ROS2 from [here](http://ci.ros2.org/view/packaging/) and follow the instructions to setup the environment.

* [Linux](https://github.com/ros2/ros2/wiki/Linux-Install-Binary)
* [macOS](https://github.com/ros2/ros2/wiki/OSX-Install-Binary)
* [Windows](https://github.com/ros2/ros2/wiki/Windows-Install-Binary)

### Install `Node.js`
**Notice:**
`rclnodejs` use some new feature of ECMAScript 6, we recommend using the latest LTS Node.js. The lowest LTS Node.js we used to verify the unit tests is `6.10.0`. Your Node.js version should not be lower than this verion.

The `Node.js` version we selected is the latest LTS [`Carbon`](https://nodejs.org/download/release/latest-carbon/) (8.x). You can install it:

* Download from Node.js offical [website](https://nodejs.org/en/), and install it.
* Use the Node Version Manager ([nvm](https://github.com/creationix/nvm)) to install it.

## Get Code

Open a terminal, and input:

```bash
git clone https://github.com/RobotWebTools/rclnodejs.git
```

then enter the folder `rclnodejs`, and get the submodule:

```bash
git submodule update --init --recursive
```

## Build Module
Before you build the module, you should make sure the ROS2 environments were loaded. You can check if the `AMENT_PREFIX_PATH` environment variable was set:

* For Windows: `echo %AMENT_PREFIX_PATH%` in the command prompt.

* For Linux and macOS: `echo $AMENT_PREFIX_PATH` in the terminal.

If the `AMENT_PREFIX_PATH` is unset, you should load the ROS2 environments:

* For Windows, open the command prompt and run

  ```
  call <path\to\ros2>\install\local_setup.bat
  ```

* For Linux and macOS, open the terminal and run:

  ```
  source <path/to/ros2>/install/local_setup.bash
  ```

This `Node.js` module is built by [node-gyp](https://www.npmjs.com/package/node-gyp), all you have to do is just to run the following command:

```javascript
npm install
```

**Windows-specific**: make sure Python 2.x interpreter is first searched in your `PATH` before running the command. You can change it temporarily by:

  ```
  set PATH=<path\to\python 2.x>;%PATH%
  ```

## Run Unit Test

[mocha](https://www.npmjs.com/package/mocha) is a javascript test framework for node.js, simply run the following command to run the unit test under `test` folder:

```javascript
npm run test
```

**Windows-specific**: the tests requires in a `Microsoft Visual Studio Native Tools command prompt`,  and also make sure Python 3.x interpreter is first searched in your `PATH` before running te test. You can change it temporarily by:
  ```
  set PATH=<path\to\python 3.x>;%PATH%
  ```

## How To Use

After building this module, you just need to follow the normal way to use it as a `Node.js` module.

```javascript
const rclnodejs = require('../index.js');

rclnodejs.init().then(() => {
  let String = rclnodejs.require('std_msgs').msg.String;
  const node = rclnodejs.createNode('publisher_example_node');
  const publisher = node.createPublisher(String, 'topic');
  let msg = new String();

  setInterval(function() {
    const str = 'Hello ROS2.0';
    msg.data = str;
    publisher.publish(msg);
  }, 1000);

  rclnodejs.spin(node);
});
```

There are also several useful examples under the `example` folder, which will show you how to use some important features, including `timer/subscription/publisher/client/service`, in `rclnodejs`. You are encouraged to try these examples to understand them.

## API Specification

The API spec is generated by `jsdoc`, you can manually run `npm run docs` to create them by yourself, or just use the existing documents under `docs` folder. To visit the on-line version, please navigate to http://robotwebtools.org/rclnodejs/docs/index.html in your browser.

## Contributing

If you want to contribute code to this project, first you need to fork the
project. The next step is to send a pull request (PR) for review. The PR will be reviewed by the project team members. Once you have gained "Look Good To Me (LGTM)", the project maintainers will merge the PR.

## License

This project abides by [Apache License 2.0](https://github.com/RobotWebTools/rclnodejs/blob/develop/LICENSE).
