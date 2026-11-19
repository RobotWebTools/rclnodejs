# Topic Examples

Run these commands from the repository root with ROS sourced and `npm install` completed. See the [build requirements](../../docs/BUILDING.md#requirements) if you need to prepare the checkout.

## Send and Receive a Message

Start the subscriber:

```bash
node example/topics/subscriber/subscription-example.mjs
```

In another ROS-sourced terminal at the repository root:

```bash
node example/topics/publisher/publisher-example.mjs
```

The publisher sends `std_msgs/msg/String` on `/topic` once a second. The subscriber prints each message it receives. Stop both with Ctrl+C.

## Other Pairs

Run one publisher and its matching subscriber. Each filename links to the source.

| Example | Publisher | Subscriber |
| --- | --- | --- |
| Strings | [Basic publisher](publisher/publisher-example.mjs) | [Basic subscriber](subscriber/subscription-example.mjs) |
| Temperature filtering | [Temperature publisher](publisher/publisher-content-filter-example.mjs) | [Filtered subscriber](subscriber/subscription-content-filter-example.mjs) |
| Joint states | [JointState publisher](publisher/publisher-message-example.mjs) | [JointState subscriber](subscriber/subscription-message-example.mjs) |
| Multidimensional arrays | [MultiArray publisher](publisher/publisher-multiarray-example.mjs) | [MultiArray subscriber](subscriber/subscription-multiarray-example.mjs) |
| QoS settings | [QoS publisher](publisher/publisher-qos-example.mjs) | [QoS subscriber](subscriber/subscription-qos-example.mjs) |
| Raw buffers | [Raw publisher](publisher/publisher-raw-message.mjs) | [Raw subscriber](subscriber/subscription-raw-message.mjs) |

Install the corresponding interface packages before generating messages: `std_msgs`, `sensor_msgs`, or `test_msgs`. Content filtering also needs a supporting RMW implementation; see the [content-filtering guide](../../tutorials/content-filtering-subscription.md).

The raw-buffer pair uses the same declared ROS type on both ends. It does not show how to construct a valid serialized message for an ordinary typed ROS subscriber.

## More Examples

- [Observable subscriber](subscriber/subscription-observable-example.mjs): several RxJS pipelines reading `/topic`; use the basic publisher.
- [Serialization modes](subscriber/subscription-serialization-modes-example.mjs) and [JSON utilities](subscriber/subscription-json-utilities-example.mjs): read `sensor_msgs/msg/LaserScan` on `/laser_scan`. Run a matching publisher separately.
- [Service-event subscriber](subscriber/subscription-service-event-example.mjs): watch `/add_two_ints/_service_event`. Start the [service](../services/service/service-example.mjs), then make a call. Service introspection requires Iron or newer.
- [Publisher validation](publisher/publisher-validation-example.mjs): check messages against their schema before publishing.
- [Name validation](validator/validator-example.mjs): check ROS node, topic, and namespace names.

For a step-by-step introduction, see [topics in the first-nodes guide](../../tutorials/writing-your-first-rclnodejs-nodes.md#topics-publishsubscribe).