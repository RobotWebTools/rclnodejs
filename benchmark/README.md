# How to Run the Benchmark tests

## Prerequisites

1.Install ROS 2 from binary

You can download the latest binary package of ROS2 from [here](http://ci.ros2.org/view/packaging/) and follow the instructions to setup the environment.

* [Linux](https://github.com/ros2/ros2/wiki/Linux-Install-Binary)
* [macOS](https://github.com/ros2/ros2/wiki/OSX-Install-Binary)
* [Windows](https://github.com/ros2/ros2/wiki/Windows-Install-Binary)

2.Install Node.js

Download the latest LTS edition from https://nodejs.org/en/

3.[Get the code](https://github.com/RobotWebTools/rclnodejs#get-code) and [install](https://github.com/RobotWebTools/rclnodejs#build-module)

## Run tests

There are 4 pairs of tests for each kind of languages, which implement the same functions.

- Two pairs of stress testing (one is publisher/subscription, the other is client/service).
- Two pairs of endurance testing (one is publisher/subscription, the other is client/service).

1.C++ benchmark tests
- Compile the source files: enter `benchmark/rclcpp/` and run `ament build .`
- The executable files locate at `build/rclcpp_benchmark/`
- Run the subscription/service first, then the publisher/client.

2.Python benchmark tests
- Enter the Python scripts folder `benchmark/rclpy/`
- Run the subscription/service first, then the publisher/client.

3.Node.js benchmark tests
- Enter the Node.js scripts folder `benchmark/rclnodejs/`
- Run the subscription/service first, then the publisher/client.

The `publisher/client` will record the time when the test started and ended.
