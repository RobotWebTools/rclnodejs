# ROS 2 Client Library Benchmarks

Performance benchmarks for comparing ROS 2 client libraries: C++ (rclcpp), Python (rclpy), and JavaScript (rclnodejs).

## Prerequisites

1. **ROS 2**: Install from [ros.org](https://docs.ros.org/en/jazzy/Installation.html)
2. **Node.js**: v16+ for rclnodejs (from [nodejs.org](https://nodejs.org/))
3. **rclnodejs**: Follow [installation guide](https://github.com/RobotWebTools/rclnodejs#installation)
4. **Build Dependencies**: For C++ benchmarks: `sudo apt install libssl-dev cmake build-essential`

## Benchmark Structure

Each client library has identical benchmark tests:

| Test Type   | Description                                 |
| ----------- | ------------------------------------------- |
| **topic**   | Publisher/subscriber performance            |
| **service** | Client/service request-response performance |

## Performance Results

### Test Environment

**Hardware:**

- **CPU:** 11th Gen Intel(R) Core(TM) i7-1185G7 @ 3.00GHz (4 cores, 8 threads)
- **Memory:** 8GB RAM
- **Architecture:** x86_64

**Software:**

- **OS:** Ubuntu 24.04.3 LTS (WSL2)
- **Kernel:** 6.6.87.2-microsoft-standard-WSL2
- **ROS 2:** Jazzy distribution
- **C++ Compiler:** GCC 13.3.0 (Ubuntu 13.3.0-6ubuntu2~24.04)
- **Python:** 3.12.3
- **Node.js:** v22.18.0

### Benchmark Results

Benchmark parameters: 1000 iterations, 1024KB message size

| Client Library          | Topic (ms) | Service (ms) | Performance Ratio   |
| ----------------------- | ---------- | ------------ | ------------------- |
| **rclcpp (C++)**        | 168        | 627          | Baseline (fastest)  |
| **rclpy (Python)**      | 1,618      | 15,380       | 9.6x / 24.5x slower |
| **rclnodejs (Node.js)** | 744        | 927          | 4.4x / 1.5x slower  |

_Last updated: August 30, 2025_

**Notes:**

- Topic benchmarks: All libraries completed successfully with 1024KB messages
- Service benchmarks: All libraries completed successfully with 1024KB responses
- Performance ratios are relative to C++ baseline
- C++ shows excellent performance as expected for a compiled language
- Node.js performs significantly better than Python, likely due to V8 optimizations
- Python service performance shows significant overhead with large payloads compared to topics

## Running Benchmarks

### C++ (rclcpp)

```bash
cd benchmark/rclcpp/
mkdir -p build && cd build
source ~/Download/ros2-linux/local_setup.bash  # Adjust path
cmake .. && make
# Run from build directory:
./publisher-stress-test -r 1000 -s 1024
./client-stress-test -r 1000
```

### Python (rclpy)

```bash
cd benchmark/rclpy/
source ~/Download/ros2-linux/local_setup.bash  # Adjust path
python3 topic/publisher-stress-test.py -r 1000 -s 1024
python3 service/client-stress-test.py -r 1000
```

### JavaScript (rclnodejs)

```bash
cd /path/to/rclnodejs/  # Project root
source ~/Download/ros2-linux/local_setup.bash  # Adjust path
node benchmark/rclnodejs/topic/publisher-stress-test.js -r 1000 -s 1024
node benchmark/rclnodejs/service/client-stress-test.js -r 1000
```

## Test Workflow

For complete tests, run subscriber/service first, then publisher/client:

**Topic Test:**

```bash
# Terminal 1: Start subscriber (adjust for your language)
python3 topic/subscription-stress-test.py          # Python
./subscription-stress-test                          # C++ (from build dir)
node benchmark/rclnodejs/topic/subscription-stress-test.js  # Node.js

# Terminal 2: Run publisher benchmark
python3 topic/publisher-stress-test.py -r 1000 -s 1024     # Python
./publisher-stress-test -r 1000 -s 1024                    # C++ (from build dir)
node benchmark/rclnodejs/topic/publisher-stress-test.js -r 1000 -s 1024  # Node.js
```

**Service Test:**

```bash
# Terminal 1: Start service (adjust for your language)
python3 service/service-stress-test.py -s 1024             # Python
./service-stress-test -s 1024                              # C++ (from build dir)
node benchmark/rclnodejs/service/service-stress-test.js -s 1024  # Node.js

# Terminal 2: Run client benchmark
python3 service/client-stress-test.py -r 1000      # Python
./client-stress-test -r 1000                       # C++ (from build dir)
node benchmark/rclnodejs/service/client-stress-test.js -r 1000  # Node.js
```

## Message Types

- **Topics**: `std_msgs/msg/UInt8MultiArray` (configurable size)
- **Services**: `nav_msgs/srv/GetMap` (map data request/response)

## Notes

- All benchmarks use high-precision timing for accurate measurements
- C++ provides baseline performance; Python/JavaScript show expected interpreted language overhead
- All implementations are now working correctly across the three client libraries
- Results vary by system configuration; use relative comparisons between client libraries
- Service benchmarks involve request-response cycles with substantial data payloads (OccupancyGrid maps)
