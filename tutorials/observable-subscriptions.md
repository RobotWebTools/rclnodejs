# Observable Subscriptions

`createObservableSubscription()` is an rclnodejs feature that wraps a regular ROS 2 subscription in an RxJS stream. It doesn't change how messages travel between nodes. For a simple message handler, `createSubscription()` is enough.

Use the project from [Writing Your First rclnodejs Nodes](writing-your-first-rclnodejs-nodes.md), with ROS sourced in both terminals. Install RxJS because the example imports it:

```bash
npm install rxjs
```

## Read Five Messages

Create `observable-subscriber.mjs`:

```javascript
import rclnodejs from 'rclnodejs';
import { take } from 'rxjs';

await rclnodejs.init();
const node = rclnodejs.createNode('observable_listener');
const source = node.createObservableSubscription(
  'std_msgs/msg/String',
  'hello'
);

source.observable.pipe(take(5)).subscribe({
  next: (message) => console.log(`Received: ${message.data}`),
  complete: () => rclnodejs.shutdown(),
});

rclnodejs.spin(node);
```

Run the publisher from the first-nodes guide in one terminal and this subscriber in another:

```bash
node publisher.mjs
```

```bash
node observable-subscriber.mjs
```

`take(5)` completes the pipeline after five messages. The completion callback shuts down the subscriber. Stop the publisher with **Ctrl+C** when finished.

## Release the Subscription

Ending a pipeline with `take()` or `unsubscribe()` does not release the ROS subscription. If the node will keep running and no other pipeline needs the source, call `source.complete()` to finish the stream and `node.destroySubscription(source.subscription)` to release the ROS subscription. In the example above, `rclnodejs.shutdown()` releases the node and its subscriptions.

See the [observable subscription example](../example/topics/subscriber/subscription-observable-example.mjs) for several pipelines sharing one source, and the [RxJS operator guide](https://rxjs.dev/guide/operators) for operator behavior.
