# Observable Subscriptions in rclnodejs

rclnodejs provides RxJS Observable support for subscriptions, enabling reactive programming patterns when working with ROS 2 messages.

## Table of Contents

- [What are Observable Subscriptions?](#what-are-observable-subscriptions)
- [Basic Usage](#basic-usage)
- [API Reference](#api-reference)
- [Examples](#examples)
- [Cleanup](#cleanup)
- [Best Practices](#best-practices)
- [Running the Examples](#running-the-examples)
- [Comparison with Callback API](#comparison-with-callback-api)

## What are Observable Subscriptions?

**Observable Subscriptions** wrap standard ROS 2 subscriptions and expose messages through RxJS Observables. This allows you to use the full power of reactive programming for:

- 🔄 **Rate limiting**: `throttleTime()`, `debounceTime()`, `auditTime()`
- 🔧 **Transformation**: `map()`, `scan()`, `buffer()`
- 🔍 **Filtering**: `filter()`, `distinctUntilChanged()`, `take()`
- 🔗 **Combining topics**: `combineLatest()`, `merge()`, `zip()`
- ⚠️ **Error handling**: `catchError()`, `retry()`, `timeout()`

Observable subscriptions are ideal for complex message processing pipelines where you need to combine, filter, or transform data from multiple sources.

> **Note:** RxJS `filter()` operates at the application level after messages are received. For simple content-based filtering that reduces network traffic, consider using [DDS Content Filtering](content-filtering-subscription.md) instead, which filters at the middleware level before messages reach your application. See [Example 6](#example-6-combining-dds-content-filtering-with-rxjs) for combined usage.

## Basic Usage

```javascript
const rclnodejs = require('rclnodejs');
const { throttleTime, map, filter } = require('rxjs');

async function main() {
  await rclnodejs.init();
  const node = new rclnodejs.Node('observable_example');

  // Create an Observable subscription
  const obsSub = node.createObservableSubscription(
    'sensor_msgs/msg/LaserScan',
    '/scan'
  );

  // Use RxJS operators to process messages
  obsSub.observable
    .pipe(
      throttleTime(200), // Limit to 5 Hz
      map((msg) => msg.ranges),
      filter((ranges) => ranges.length > 0)
    )
    .subscribe((ranges) => {
      console.log('Received ranges:', ranges.length);
    });

  node.spin();
}

main();
```

## API Reference

### `node.createObservableSubscription(typeClass, topic, options?, eventCallbacks?)`

Creates a subscription that returns an `ObservableSubscription`.

**Parameters:**

- `typeClass` - Message type (string, object, or class)
- `topic` - Topic name
- `options` - Optional subscription options (same as `createSubscription()`)
- `eventCallbacks` - Optional event callbacks

**Returns:** `ObservableSubscription`

### `ObservableSubscription`

| Property/Method | Type            | Description                               |
| --------------- | --------------- | ----------------------------------------- |
| `observable`    | `Observable<T>` | RxJS Observable that emits messages       |
| `subscription`  | `Subscription`  | Underlying ROS 2 subscription             |
| `topic`         | `string`        | Topic name                                |
| `isDestroyed`   | `boolean`       | Whether the observable has been completed |
| `complete()`    | `void`          | Complete the observable and stop emitting |
| `destroy()`     | `void`          | Alias for `complete()`                    |

## Examples

### Example 1: Throttling High-Frequency Sensors

```javascript
const { throttleTime } = require('rxjs');

// LiDAR publishes at 20Hz, but we only need 5Hz for visualization
const lidarSub = node.createObservableSubscription(
  'sensor_msgs/msg/LaserScan',
  '/scan'
);

lidarSub.observable
  .pipe(throttleTime(200)) // 200ms = 5Hz
  .subscribe((scan) => {
    visualize(scan);
  });
```

### Example 2: Combining Multiple Topics

```javascript
const { combineLatest, map } = require('rxjs');

const odomSub = node.createObservableSubscription(
  'nav_msgs/msg/Odometry',
  '/odom'
);
const imuSub = node.createObservableSubscription('sensor_msgs/msg/Imu', '/imu');

combineLatest([odomSub.observable, imuSub.observable])
  .pipe(
    map(([odom, imu]) => ({
      position: odom.pose.pose.position,
      orientation: imu.orientation,
    }))
  )
  .subscribe((combined) => {
    console.log('Combined data:', combined);
  });
```

### Example 3: Debouncing Burst Events

```javascript
const { debounceTime } = require('rxjs');

// Joystick commands may come in bursts - only act on the final value
const joySub = node.createObservableSubscription('sensor_msgs/msg/Joy', '/joy');

joySub.observable
  .pipe(debounceTime(50)) // Wait 50ms of quiet before processing
  .subscribe((joy) => {
    processJoystickCommand(joy);
  });
```

### Example 4: Sampling Every Nth Message

```javascript
const { filter } = require('rxjs');

// IMU at 100Hz - sample every 10th message for logging
let count = 0;
const imuSub = node.createObservableSubscription('sensor_msgs/msg/Imu', '/imu');

imuSub.observable.pipe(filter(() => ++count % 10 === 0)).subscribe((imu) => {
  logToFile(imu);
});
```

### Example 5: Buffering Messages

```javascript
const { bufferTime, filter } = require('rxjs');

const tempSub = node.createObservableSubscription(
  'sensor_msgs/msg/Temperature',
  '/temperature'
);

// Collect messages over 1 second, then process as batch
tempSub.observable
  .pipe(
    bufferTime(1000),
    filter((batch) => batch.length > 0)
  )
  .subscribe((batch) => {
    const avgTemp =
      batch.reduce((sum, msg) => sum + msg.temperature, 0) / batch.length;
    console.log('Average temperature:', avgTemp);
  });
```

### Example 6: Combining DDS Content Filtering with RxJS

For optimal performance, use DDS content filtering to reduce network traffic at the middleware level, then apply RxJS operators for additional processing:

```javascript
const { throttleTime, map } = require('rxjs');

// DDS filters at middleware level - only temperatures > 30°C are delivered
const tempSub = node.createObservableSubscription(
  'sensor_msgs/msg/Temperature',
  '/temperature',
  {
    contentFilter: {
      expression: 'temperature > %0',
      parameters: ['30.0'],
    },
  }
);

// RxJS processes the pre-filtered stream
tempSub.observable
  .pipe(
    throttleTime(1000), // Rate limit to 1 msg/sec
    map((msg) => ({
      celsius: msg.temperature,
      fahrenheit: msg.temperature * 1.8 + 32,
    }))
  )
  .subscribe((temp) => {
    console.log(`High temp alert: ${temp.celsius}°C (${temp.fahrenheit}°F)`);
  });
```

This approach provides:

- **Network efficiency**: DDS drops messages below 30°C before transmission
- **CPU efficiency**: RxJS only processes relevant messages
- **Flexibility**: RxJS handles rate limiting and transformation

> See [Content Filtering Subscription](content-filtering-subscription.md) for more details on DDS content filtering.

## Cleanup

Always clean up subscriptions when done:

```javascript
// Option 1: Complete the observable
obsSub.complete();

// Option 2: Destroy via node
node.destroySubscription(obsSub.subscription);

// Option 3: Destroy the entire node
node.destroy();
```

## Best Practices

### 1. Always Unsubscribe

Prevent memory leaks by unsubscribing when done:

```javascript
const rxjsSubscription = obsSub.observable
  .pipe(throttleTime(100))
  .subscribe((msg) => console.log(msg));

// Later, when cleanup is needed:
rxjsSubscription.unsubscribe();
obsSub.complete();
```

### 2. Use `take()` for Finite Streams

```javascript
const { take } = require('rxjs');

// Only process the first 10 messages
obsSub.observable.pipe(take(10)).subscribe({
  next: (msg) => console.log(msg),
  complete: () => console.log('Received 10 messages'),
});
```

### 3. Handle Errors Gracefully

```javascript
const { catchError } = require('rxjs');
const { of } = require('rxjs');

obsSub.observable
  .pipe(
    map((msg) => processMessage(msg)),
    catchError((err) => {
      console.error('Processing error:', err);
      return of(null); // Continue with null on error
    }),
    filter((result) => result !== null)
  )
  .subscribe((result) => {
    console.log('Processed:', result);
  });
```

### 4. Combine with Content Filtering

For maximum efficiency, combine RxJS operators with DDS-level content filtering:

```javascript
const { map } = require('rxjs');

// DDS filters at middleware level (reduces network traffic)
const obsSub = node.createObservableSubscription(
  'sensor_msgs/msg/Temperature',
  '/temperature',
  {
    contentFilter: {
      expression: 'temperature > %0',
      parameters: [50.0],
    },
  }
);

// RxJS operators for additional processing
obsSub.observable
  .pipe(
    map((msg) => ({ temp: msg.temperature, critical: msg.temperature > 80 }))
  )
  .subscribe((data) => {
    if (data.critical) {
      console.warn('Critical temperature:', data.temp);
    }
  });
```

## Running the Examples

The rclnodejs repository includes an Observable subscription example in `example/topics/subscriber/`.

### Run the Built-in Example

```bash
# Terminal 1 - Start a publisher
node example/topics/publisher/publisher-example.js

# Terminal 2 - Run the observable subscription example
node example/topics/subscriber/subscription-observable-example.js
```

### Expected Output

```
Observable subscription created on /topic
Run: ros2 topic pub /topic std_msgs/msg/String "{data: Hello ROS}" -r 5
[Throttled] Hello ROS 2 from rclnodejs
[Filtered] Hello ROS 2 from rclnodejs
[Throttled] Hello ROS 2 from rclnodejs
[Filtered] Hello ROS 2 from rclnodejs
[Batched] 3 messages
[Filtered] Hello ROS 2 from rclnodejs
[Throttled] Hello ROS 2 from rclnodejs
[Filtered] Hello ROS 2 from rclnodejs
[Batched] 3 messages
```

- **[Throttled]** — Rate limited to ~2 messages/second via `throttleTime(500)`
- **[Filtered]** — Only messages containing "ROS" (all pass in this case)
- **[Batched]** — Emits after every 3 messages via `bufferCount(3)`

### Custom Example

```javascript
// observable-example.js
const rclnodejs = require('rclnodejs');
const { take, map } = require('rxjs');

async function main() {
  await rclnodejs.init();
  const node = new rclnodejs.Node('observable_demo');

  const obsSub = node.createObservableSubscription(
    'std_msgs/msg/String',
    '/test_topic'
  );

  obsSub.observable
    .pipe(
      take(5),
      map((msg) => msg.data.toUpperCase())
    )
    .subscribe({
      next: (data) => console.log('Received:', data),
      complete: () => {
        console.log('Done - received 5 messages');
        node.destroy();
        rclnodejs.shutdown();
      },
    });

  node.spin();
}

main().catch(console.error);
```

```bash
# Terminal 2 - Run the example
node observable-example.js
```

### Expected Output

```
Received: HELLO
Received: HELLO
Received: HELLO
Received: HELLO
Received: HELLO
Done - received 5 messages
```

## Comparison with Callback API

| Feature          | `createSubscription()` | `createObservableSubscription()` |
| ---------------- | ---------------------- | -------------------------------- |
| Style            | Callback-based         | Observable-based                 |
| Rate limiting    | Manual implementation  | Via RxJS operators               |
| Combining topics | Manual                 | Built-in with RxJS               |
| Learning curve   | Lower                  | Requires RxJS knowledge          |
| Use case         | Simple subscriptions   | Complex reactive pipelines       |
| Dependencies     | None                   | RxJS (included)                  |

Both APIs can coexist in the same application. Use the callback-based API for simple cases and the Observable API when you need advanced reactive patterns.

---

_This tutorial is part of the rclnodejs documentation. For more tutorials and examples, visit the [rclnodejs GitHub repository](https://github.com/RobotWebTools/rclnodejs)._
