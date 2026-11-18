# Observable Subscriptions

An observable subscription exposes ROS messages as an RxJS stream. Use it when operators make the processing clearer than a callback, such as combining topics or grouping messages into batches. A plain `createSubscription()` callback is still a good choice for simple handlers.

Use the project from [Writing Your First rclnodejs Nodes](writing-your-first-rclnodejs-nodes.md), with ROS sourced. Install RxJS as a direct dependency because the examples import it:

```bash
npm install rxjs
```

## Process a Message Stream

Create `observable-subscriber.mjs`:

```javascript
import rclnodejs from 'rclnodejs';
import { filter, map, take, throttleTime } from 'rxjs';

await rclnodejs.init();
const node = rclnodejs.createNode('observable_listener');
const source = node.createObservableSubscription(
  'std_msgs/msg/String',
  'hello'
);

source.observable
  .pipe(
    filter((message) => message.data.includes('ROS')),
    throttleTime(500),
    map((message) => message.data.toUpperCase()),
    take(5)
  )
  .subscribe({
    next: (text) => console.log(`Received: ${text}`),
    error: (error) => {
      console.error(error);
      source.complete();
      node.destroySubscription(source.subscription);
      rclnodejs.shutdown();
    },
    complete: () => {
      console.log('Stream complete');
      source.complete();
      node.destroySubscription(source.subscription);
      rclnodejs.shutdown();
    },
  });

console.log('Listening on /hello');
rclnodejs.spin(node);
```

Run the publisher from the first-nodes guide in one terminal and this subscriber in another:

```bash
node publisher.mjs
```

```bash
node observable-subscriber.mjs
```

The subscriber prints five matching messages in uppercase, then shuts down. For a faster stream, use the CLI instead of the publisher:

```bash
ros2 topic pub /hello std_msgs/msg/String '{data: "Hello ROS"}' --rate 10
```

`throttleTime(500)` forwards the first matching message, then drops others during the next 500 ms. It limits the output rate, not the publisher's rate or the number of messages received by ROS. `take(5)` counts messages after the preceding operators and completes that RxJS pipeline.

Stop the publisher with **Ctrl+C** when finished.

## Combine Topics

`combineLatest()` waits for one message from each input. After that, a message on either topic emits a new pair, even if its value is unchanged.

For example, add this pipeline before `spin()` in a node that receives two numeric topics:

```javascript
import { combineLatest } from 'rxjs';

const left = node.createObservableSubscription('std_msgs/msg/Float64', 'left');
const right = node.createObservableSubscription(
  'std_msgs/msg/Float64',
  'right'
);

const combined = combineLatest([left.observable, right.observable]).subscribe(
  ([leftMessage, rightMessage]) => {
    console.log(`Total: ${leftMessage.data + rightMessage.data}`);
  }
);
```

This combines the most recent values; it does not synchronize sensor timestamps. If time alignment matters, implement that explicitly rather than assuming two messages arrived together.

## Release the Subscription

There are three separate resources to consider:

- The RxJS subscription returned by `subscribe()`. Calling `unsubscribe()` stops that pipeline.
- The observable wrapper returned by `createObservableSubscription()`. Calling `complete()` notifies its observers and stops further emissions; `destroy()` is an alias.
- The underlying ROS subscription, available as `.subscription`. Call `node.destroySubscription()` to unregister and release it.

For the combined pipeline above, cleanup while the node is still alive looks like this:

```javascript
combined.unsubscribe();
left.complete();
right.complete();
node.destroySubscription(left.subscription);
node.destroySubscription(right.subscription);
```

Neither `take()` nor `unsubscribe()` automatically releases the ROS subscription. Completing a wrapper also does not release it. Destroying the node releases its ROS entities, but does not itself call `complete()` on the observable wrapper.

## Operators and Errors

All observers of one wrapper share the same underlying ROS subscription. The stream is live: a new observer does not receive messages that arrived before it subscribed.

`filter()` and other RxJS operators run after ROS has received and converted a message. For middleware-level filtering, see [Content Filtering](content-filtering-subscription.md).

An exception in an operator ends that pipeline unless you handle it. `catchError(() => of(null))` replaces the failed stream with a single value and then completes; it does not resume the original ROS stream. Use an error strategy that matches your application, and remember that retrying an observer cannot repair a destroyed ROS subscription.

See the [observable subscription example](../example/topics/subscriber/subscription-observable-example.mjs) for several pipelines sharing one source, and the [RxJS operator guide](https://rxjs.dev/guide/operators) for operator behavior.
