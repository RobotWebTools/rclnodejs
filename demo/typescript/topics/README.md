# TypeScript Topics Demo

The [publisher](src/publisher.ts) sends `std_msgs/msg/String` messages on `/ts_demo`, and the [subscriber](src/subscriber.ts) prints them. Both use a type name and a constructor from `rclnodejs.require()`.

## Run

After [TypeScript setup](../README.md#setup), run from the repository root:

```bash
cd demo/typescript/topics
npm install --ignore-scripts
npm run start:both
```

`start:both` builds and runs both nodes. To run them separately, start `npm run start:publisher`, then `npm run start:subscriber` in another terminal in the demo directory. Stop them with Ctrl+C.

## Inspect the Topic

```bash
ros2 topic echo /ts_demo std_msgs/msg/String
```

The timer fires once a second. Set `PUBLISH_INTERVAL` to `500_000_000n` for 500 ms. `createTimer()` takes bigint nanoseconds.