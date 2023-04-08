# Build rclnodejs

### Get ready for ROS 2

1.Install ROS 2 from binary package.

ROS 2 is a cross-platform system, which covers Linux, macOS and Windows, and the `rclnodejs` module is developed against the [`master`](https://github.com/ros2/ros2/blob/master/ros2.repos) branch of ROS 2. You can download the latest binary packages from [ROS 2 build farm](http://ci.ros2.org/view/packaging/) and follow the instructions of [Linux](https://index.ros.org/doc/ros2/Installation/Linux-Install-Binary/)/[macOS](https://index.ros.org/doc/ros2/Installation/OSX-Install-Binary/)/[Windows](https://index.ros.org/doc/ros2/Installation/Windows-Install-Binary/) to setup the environment (If you want to run your apps on a stable release of ROS 2, e.g. crystal-clemmys, please see the section `Running on Stable Release of ROS 2).

2.Build ROS 2 from scratch.

Alternatively, you can build ROS 2 from scratch. Please select the platform you want to work on, then reference the instructions of [Linux](https://index.ros.org/doc/ros2/Installation/Linux-Development-Setup/)/[macOS](https://index.ros.org/doc/ros2/Installation/OSX-Development-Setup/)/[Windows](https://index.ros.org/doc/ros2/Installation/Windows-Development-Setup/) to build ROS 2 (please build with flag `--merge-install`).

### Install `Node.js`

**Notice:**
`rclnodejs` should only be used with node versions between 10.23.1 - 19.x. The lowest LTS Node.js we used to verify the unit tests is `10.23.1`.    

I install Nodejs from either:

- Node.js official [website](https://nodejs.org/en/)
- Node Version Manager ([nvm](https://github.com/creationix/nvm))

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
