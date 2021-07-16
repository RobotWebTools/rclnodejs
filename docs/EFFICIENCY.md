# Tips for efficent use of rclnodejs
While our benchmarks place rclnodejs performance at or above that of [rclpy](https://github.com/ros2/rclpy) we recommend appyling efficient coding and configuration practices where applicable. 

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
