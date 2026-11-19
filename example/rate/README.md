# Rate Example

The [rate example](rate-example.mjs) publishes `std_msgs/msg/String` on `/topic` roughly every 10 ms. Its main loop waits on a 0.5 Hz rate, then calls `spinOnce()` to process ready callbacks.

## Run

With ROS sourced and `npm install` completed, run from the repository root:

```bash
node example/rate/rate-example.mjs
```

You need `std_msgs`; see the [build requirements](../../docs/BUILDING.md#requirements) for checkout setup. The subscriber prints receipt and publication timestamps about every two seconds.

To watch the publisher independently, run this in another ROS-sourced terminal:

```bash
ros2 topic echo /topic std_msgs/msg/String
```

Stop both commands with Ctrl+C.

## What Controls the Rate

`await rate.sleep()` pauses the calling loop, not the JavaScript event loop. The publisher's `setInterval()` can keep running while the loop waits. `spinOnce(node, 1000)` then handles ready callbacks, waiting up to one second if none are ready; it does not spend one second continuously processing messages.

This example limits how often it processes callbacks, not which numbered message it receives. DDS history, queue depth, and scheduling affect the samples delivered. There is no guarantee of receiving exactly every 200th message or meeting a hard real-time deadline.