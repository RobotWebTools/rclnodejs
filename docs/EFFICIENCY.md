# Tips for efficent use of rclnodejs

While our benchmarks place rclnodejs performance at or above that of [rclpy](https://github.com/ros2/rclpy) we recommend applying efficient coding and configuration practices where applicable.

## Tip-1: Disable Parameter Services

The typical ROS 2 node creation process includes creating an internal parameter service who's job is to fulfill requests for parameter meta-data and to set and update node parameters. If your ROS 2 node does not support public parameters then you can save the resources consumed by the parameter service. Disable the node parameter service by setting the `NodeOption.startParameterServices` property to false as shown below:

```
let options = new NodeOptions(false);
// or
options.startParameterServices = false;

let node = new Node(nodeName, namespace, Context.defaultContext(), options);
```

## Tip-2: Disable LifecycleNode Lifecycle Services

The LifecycleNode constructor creates 5 life-cycle services to support the ROS 2 lifecycle specification. If your LifecycleNode instance will not be operating in a managed-node context consider disabling the lifecycle services via the LifecycleNode constructor as shown:

```
let enableLifecycleCommInterface = false;

let node = new LifecycleNode(
  nodeName,
  namespace,
  Context.defaultContext,
  NodeOptions.defaultOptions,
  enableLifecycleCommInterface
);
```

## Tip-3: Use Content-filtering Subscriptions

The ROS Humble release introduced content-filtering topics
which enable a subscription to limit the messages it receives
to a subset of interest. While the application of the a content-filter
is specific to the DDS/RMW vendor, the general approach is to apply
filtering on the publisher side. This can reduce network bandwidth
for pub-sub communications and message processing and memory
overhead of rclnodejs nodes.

Note: Be sure to confirm that your RMW implementation supports
content-filter before attempting to use it. In cases where content-filtering
is not supported your Subscription will simply ignore your filter and
continue operating with no filtering.

Example:

```
  // create a content-filter to limit incoming messages to
  // only those with temperature > 75C.
  const options = rclnodejs.Node.getDefaultOptions();
  options.contentFilter = {
    expression: 'temperature > %0',
    parameters: [75],
  };

  node.createSubscription(
    'sensor_msgs/msg/Temperature',
    'temperature',
    options,
    (temperatureMsg) => {
      console.log(`EMERGENCY temperature detected: ${temperatureMsg.temperature}`);
    }
  );

```
