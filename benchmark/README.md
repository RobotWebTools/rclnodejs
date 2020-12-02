# Benchmarks for ROS 2.0 clients

This folder contains code used to measure the performance between different ROS 2.0 clients.

## Prerequisites

1.Install ROS 2.0 from binary

You can download the latest binary package of ROS2 from [here](http://ci.ros2.org/view/packaging/) and follow the instructions to setup the environment.

* [Linux](https://github.com/ros2/ros2/wiki/Linux-Install-Binary)
* [macOS](https://github.com/ros2/ros2/wiki/OSX-Install-Binary)
* [Windows](https://github.com/ros2/ros2/wiki/Windows-Install-Binary)

2.Install Node.js

Download the latest LTS edition from https://nodejs.org/en/

3.[Get the code](https://github.com/RobotWebTools/rclnodejs#get-code) and [install](https://github.com/RobotWebTools/rclnodejs#build-module)

## Benchmark directories

The table lists the directories for each kind of the ROS 2.0 clients.

Directory | Purpose |
:----------:| ------------- |
topic | Benchmarks for `publisher` and `subscription` features
service | Benchmarks for `client` and `service` features
startup | Benchamrks for measuring the startup time consumption

## Run tests

1.Benchmark for [rclcpp](https://github.com/ros2/rclcpp)
- Compile the source files, `cd benchmark/rclcpp/` and run `colcon build`
- The executable files locate at `build/rclcpp_benchmark/`
- `your_benchamrk -h` for help.

2.Benchmark for [rclpy](https://github.com/ros2/rclpy)
- Enter the Python scripts folder `benchmark/rclpy/`
- `python3 your_benchamrk -h` for help.

3.Benchmark for rclnodejs
- Enter the Node.js scripts folder `benchmark/rclnodejs/`
- `node your_benchamrk -h` for help.
