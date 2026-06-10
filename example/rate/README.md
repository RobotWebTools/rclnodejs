# ROS 2 Rate Examples

This directory contains examples demonstrating ROS 2 rate control using rclnodejs. Rate control allows you to manage the timing and frequency of operations in ROS 2 applications, providing precise control over loop execution rates.

## Overview

ROS 2 rate control provides mechanisms for:

- **Precise Timing**: Control loop execution frequency with high precision
- **Rate Limiting**: Prevent operations from exceeding desired frequencies
- **Synchronization**: Coordinate timing across different parts of an application
- **Performance Control**: Manage computational load and resource usage
- **Real-time Behavior**: Achieve predictable timing for time-critical applications

Rate control is essential for:

- Sensor data processing at specific frequencies
- Control loops requiring precise timing
- Throttling high-frequency operations
- Synchronizing multiple processes
- Managing system resource usage

## Rate Example

### Rate-Limited Loop with High-Frequency Publisher (`rate-example.cjs`)

**Purpose**: Demonstrates rate control in a scenario with mismatched publication and processing frequencies.

#### Functionality

This example creates a sophisticated timing demonstration that shows:

1. **High-Frequency Publisher**: Publishes messages every 10ms (100 Hz)
2. **Rate-Limited Subscriber**: Processes messages at 0.5 Hz (every 2 seconds)
3. **Message Filtering**: Demonstrates how rate limiting affects message processing
4. **Timing Visualization**: Shows timestamps to illustrate timing differences

#### Architecture

- **Node Name**: `test_node`
- **Topic**: `topic`
- **Message Type**: `std_msgs/msg/String`
- **Publisher Rate**: 100 Hz (every 10ms)
- **Processing Rate**: 0.5 Hz (every 2 seconds)
- **Rate Ratio**: 1:200 (subscriber sees every 200th published message)

#### Key Components

1. **Publisher**: Uses `setInterval()` for high-frequency publishing
2. **Subscriber**: Receives all messages but processes at controlled rate
3. **Rate Object**: Controls main loop execution frequency
4. **Spin Control**: Processes callbacks at rate-limited intervals

#### Features Demonstrated

- **Rate Creation**: Using `node.createRate(frequency)`
- **Rate Sleep**: Blocking execution with `await rate.sleep()`
- **Spin Control**: Using `spinOnce()` for controlled callback processing
- **Frequency Mismatch**: Handling different publish/process rates
- **Timestamp Tracking**: Monitoring message timing and processing delays

#### Run Command

```bash
node rate-example.cjs
```

## Sample Output

When you run the rate example, you'll see output showing the rate-limited message processing:

### Expected Output Pattern

```
Received(1642694400123): hello 1642694400012
Received(1642694402134): hello 1642694402089
Received(1642694404145): hello 1642694404123
Received(1642694406156): hello 1642694406134
...
```

### Output Analysis

- **Received Timestamp**: When the subscriber processed the message (every ~2 seconds)
- **Message Timestamp**: When the message was originally published
- **Time Gap**: Shows the delay between publication and processing
- **Missing Messages**: Only every ~200th message is processed due to rate limiting

### Monitoring All Messages

To see all published messages (not just the rate-limited ones), run:

```bash
ros2 topic echo topic std_msgs/msg/String
```

This will show the high-frequency stream:

```
data: hello 1642694400012
---
data: hello 1642694400022
---
data: hello 1642694400032
---
... (continuing every 10ms)
```

## Rate Control Implementation Details

### Rate Object Creation

```javascript
// Create a rate object for 0.5 Hz (once every 2 seconds)
const rate = await node.createRate(0.5);
```

Rate frequencies can be specified as:

- **Hertz (Hz)**: Cycles per second (e.g., `0.5` = once every 2 seconds)
- **Common Frequencies**: `1.0` (1 Hz), `10.0` (10 Hz), `100.0` (100 Hz)

### Rate-Limited Loop Pattern

```javascript
let forever = true;
while (forever) {
  await rate.sleep(); // Wait for next rate cycle
  rclnodejs.spinOnce(node, 1000); // Process callbacks for up to 1000ms
}
```

### Key Rate Control Concepts

#### Rate Sleep Behavior

- **Blocking**: `rate.sleep()` blocks execution until the next rate cycle
- **Precise Timing**: Compensates for processing time to maintain exact frequency
- **Async/Await**: Uses modern JavaScript patterns for clean asynchronous code

#### Spin Control Integration

- **spinOnce()**: Processes callbacks once with optional timeout
- **Rate Coordination**: Combines with rate limiting for controlled processing
- **Non-blocking**: Allows rate control without blocking other operations

## Advanced Rate Control Patterns

### Multiple Rate Objects

```javascript
// Different rates for different operations
const fastRate = await node.createRate(10.0); // 10 Hz
const slowRate = await node.createRate(1.0); // 1 Hz

// Use appropriate rate for each operation
while (running) {
  if (needsFastProcessing) {
    await fastRate.sleep();
    processFastData();
  } else {
    await slowRate.sleep();
    processSlowData();
  }
  rclnodejs.spinOnce(node);
}
```

### Conditional Rate Control

```javascript
const rate = await node.createRate(baseFrequency);

while (running) {
  await rate.sleep();

  if (systemLoad < threshold) {
    // Process more data when system load is low
    processData();
  }

  rclnodejs.spinOnce(node);
}
```

### Rate with Timer Integration

```javascript
// Combine rate control with timers for complex timing
const rate = await node.createRate(1.0);
const timer = node.createTimer(BigInt(100000000), () => {
  // High-frequency timer operation
  publishSensorData();
});

while (running) {
  await rate.sleep();
  // Low-frequency control operation
  updateControlParameters();
  rclnodejs.spinOnce(node);
}
```

## Rate Control Use Cases

### Sensor Data Processing

```javascript
const sensorRate = await node.createRate(50.0); // 50 Hz sensor processing

while (running) {
  await sensorRate.sleep();

  // Process sensor data at controlled rate
  const sensorData = readSensors();
  const processedData = processSensorData(sensorData);
  publishProcessedData(processedData);

  rclnodejs.spinOnce(node);
}
```

### Control Loops

```javascript
const controlRate = await node.createRate(100.0); // 100 Hz control loop

while (running) {
  await controlRate.sleep();

  // Execute control algorithm at precise frequency
  const currentState = getCurrentState();
  const controlCommand = computeControl(currentState, setpoint);
  sendControlCommand(controlCommand);

  rclnodejs.spinOnce(node);
}
```

### Data Throttling

```javascript
const throttleRate = await node.createRate(2.0); // Throttle to 2 Hz

const dataBuffer = [];

// High-frequency data collection
setInterval(() => {
  dataBuffer.push(collectData());
}, 1); // Collect every 1ms

// Rate-limited data processing
while (running) {
  await throttleRate.sleep();

  if (dataBuffer.length > 0) {
    const batch = dataBuffer.splice(0, 100); // Process in batches
    processBatch(batch);
  }

  rclnodejs.spinOnce(node);
}
```

## Performance Considerations

### Rate Accuracy

- Rate objects maintain precise timing by compensating for processing delays
- Long processing times may cause rate drift or missed cycles
- Monitor actual execution frequency vs. target frequency

### Resource Management

```javascript
// Monitor rate performance
const startTime = Date.now();
let cycleCount = 0;

const rate = await node.createRate(10.0);

while (running) {
  await rate.sleep();

  // Your processing here
  processData();

  cycleCount++;

  // Check actual frequency periodically
  if (cycleCount % 100 === 0) {
    const elapsed = (Date.now() - startTime) / 1000;
    const actualRate = cycleCount / elapsed;
    console.log(`Target: 10 Hz, Actual: ${actualRate.toFixed(2)} Hz`);
  }

  rclnodejs.spinOnce(node);
}
```

### Memory and CPU Usage

- Rate objects have minimal overhead
- `spinOnce()` timeout prevents indefinite blocking
- Consider processing time relative to rate period

## Comparing Rate Control Methods

| Method          | Use Case                   | Precision | Complexity |
| --------------- | -------------------------- | --------- | ---------- |
| `createRate()`  | Precise frequency control  | High      | Low        |
| `setInterval()` | Simple periodic operations | Medium    | Very Low   |
| `createTimer()` | ROS-integrated timing      | High      | Low        |
| `setTimeout()`  | One-time delays            | Low       | Very Low   |

### When to Use Rate Control

- **High-precision timing requirements**
- **Frequency synchronization across operations**
- **Rate limiting for resource management**
- **Control loop implementation**
- **Real-time system behavior**

### When to Use Alternatives

- **Simple periodic publishing**: Use `createTimer()`
- **One-time operations**: Use `setTimeout()`
- **Event-driven processing**: Use callbacks without rate control

## Troubleshooting

### Common Issues

1. **Rate Drift**:
   - Processing time exceeds rate period
   - Solution: Optimize processing or reduce rate frequency
   - Monitor actual vs. target frequency

2. **Missed Messages**:
   - Rate limiting causes message drops
   - Expected behavior in the example (only every 200th message processed)
   - Use `ros2 topic echo` to see all messages

3. **High CPU Usage**:
   - Rate loop running too fast for processing capacity
   - Solution: Reduce rate frequency or optimize processing
   - Monitor system resource usage

4. **Irregular Timing**:
   - System load affecting rate precision
   - Solution: Use real-time operating system or lower rates
   - Consider process priority settings

### Debugging Rate Control

```javascript
const rate = await node.createRate(10.0);
let lastTime = Date.now();

while (running) {
  await rate.sleep();

  const currentTime = Date.now();
  const interval = currentTime - lastTime;
  console.log(`Rate interval: ${interval}ms (target: 100ms)`);
  lastTime = currentTime;

  rclnodejs.spinOnce(node);
}
```

### Monitoring Rate Performance

- Use timing logs to verify rate accuracy
- Monitor CPU and memory usage
- Check for rate drift over long periods
- Validate that critical timing requirements are met

## Notes

- Rate control provides precise timing for predictable system behavior
- The example demonstrates intentional rate mismatch for educational purposes
- In production, match rates to system requirements and capabilities
- Rate objects automatically compensate for processing delays
- `spinOnce()` timeout prevents indefinite blocking on callback processing
- Rate control is essential for real-time and control applications
- Consider system limitations when choosing target frequencies
- Use monitoring to verify actual vs. target performance
