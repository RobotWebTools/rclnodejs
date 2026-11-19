# ROS 2 Client Library Benchmarks

Topic and service benchmarks for rclcpp (C++), rclpy (Python), and rclnodejs (JavaScript).

## Prerequisites

- Install ROS 2 with rclcpp, rclpy, `std_msgs`, and `nav_msgs`.
- For JavaScript, prepare the checkout using the [build guide](../docs/BUILDING.md).
- For C++, install `cmake`, `build-essential`, and `libssl-dev`.

Run from the repository root. Source ROS 2 in each terminal:

```bash
source /opt/ros/<distro>/setup.bash
```

Build the C++ benchmarks:

```bash
cmake -S benchmark/rclcpp -B benchmark/rclcpp/build
cmake --build benchmark/rclcpp/build
```

## Recorded Results

Recorded on August 30, 2025, using rclnodejs v1.5.0. These are historical results, not measurements of the current release.

### Test Environment

Hardware:

- CPU: 11th Gen Intel(R) Core(TM) i7-1185G7 @ 3.00GHz (4 cores, 8 threads)
- Memory: 8GB RAM
- Architecture: x86_64

Software:

- OS: Ubuntu 24.04.3 LTS (WSL2)
- Kernel: 6.6.87.2-microsoft-standard-WSL2
- ROS 2: [Jazzy Patch Release 6](https://github.com/ros2/ros2/releases/tag/release-jazzy-20250820)
- C++ compiler: GCC 13.3.0 (Ubuntu 13.3.0-6ubuntu2~24.04)
- Python: 3.12.3
- Node.js: v22.18.0
- rclnodejs: [v1.5.0](https://www.npmjs.com/package/rclnodejs/v/1.5.0)

### Benchmark Results

1000 iterations with 1024KB topic payloads and service responses. Ratios are relative to the C++ result.

| Client Library          | Topic (ms) | Service (ms) | Performance Ratio   |
| ----------------------- | ---------- | ------------ | ------------------- |
| rclcpp (C++)           | 168        | 627          | Baseline           |
| rclpy (Python)         | 1,618      | 15,380       | 9.6x / 24.5x slower |
| rclnodejs (Node.js)     | 744        | 927          | 4.4x / 1.5x slower  |

## Running Benchmarks

Choose one implementation at a time. Start its subscriber or service in terminal 1, then run its publisher or client in terminal 2.

### Topics

Publishes `std_msgs/msg/UInt8MultiArray`; `-s` sets the array size in KB.

| Library | Terminal 1: subscriber | Terminal 2: publisher |
| --- | --- | --- |
| rclcpp | `benchmark/rclcpp/build/subscription-stress-test` | `benchmark/rclcpp/build/publisher-stress-test -r 1000 -s 1024` |
| rclpy | `python3 benchmark/rclpy/topic/subscription-stress-test.py` | `python3 benchmark/rclpy/topic/publisher-stress-test.py -r 1000 -s 1024` |
| rclnodejs | `node benchmark/rclnodejs/topic/subscription-stress-test.js` | `node benchmark/rclnodejs/topic/publisher-stress-test.js -r 1000 -s 1024` |

### Services

Calls `nav_msgs/srv/GetMap`; `-s` sets the service's map data size in KB.

| Library | Terminal 1: service | Terminal 2: client |
| --- | --- | --- |
| rclcpp | `benchmark/rclcpp/build/service-stress-test -s 1024` | `benchmark/rclcpp/build/client-stress-test -r 1000` |
| rclpy | `python3 benchmark/rclpy/service/service-stress-test.py -s 1024` | `python3 benchmark/rclpy/service/client-stress-test.py -r 1000` |
| rclnodejs | `node benchmark/rclnodejs/service/service-stress-test.js -s 1024` | `node benchmark/rclnodejs/service/client-stress-test.js -r 1000` |

### What the Timers Measure

Timing differs between runners. The JavaScript topic timer covers node, publisher, and message setup, then publishing, not subscriber delivery or round-trip latency. The service timer also includes node/client setup and service discovery.

These are total run times, not per-message latencies. Middleware, payload size, and hardware affect the results.
