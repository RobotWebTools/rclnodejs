# Introduction
This directory contains some examples based on `rclnodejs`.

## Precondition
To run the examples, you should make sure:
* `rclnodejs` is intalled. To see how to install it, see the repository [README](../README.md)
* In each terminal session, the ROS2 environment is loaded. To check if it's loaded, use command `echo $AMENT_PREFIX_PATH` on POSIX systems and `echo %AMENT_PREFIX_PATH%` on Windows.

## Run the client/service and publisher/subscription examples
For these examples, each of them contains a pair of JavaScript files: `client/service, publisher/subscription`, you should always run the `service` or `subscription` node first:

In terminal 1:
```
$ node subscription-example.js
```

In temrinal 2:
```
$ node publisher-example.js
```

You can press `CTRL + C` to shutdown them.

## Run the action example
**Notice**: the action example requires the `ros1_action` package to be compiled, this package is located in `test` directory. To compile this package:
```
$ rm -fr generated
$ export AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH:$(pwd)/test/ros1_actions
$ node scripts/generate_messages.js
$ cd test/ros1_action
$ colcon build
$ cd ../../
```

After the build, you can run the action example:
In terminal 1:
```
$ source test/ros1_actions/install/local_setup.bash
$ node example/action-server-example.js
```

In terminal 2:
```
$ source test/ros1_actions/install/local_setup.bash
$ node example/action-client-example.js
```
