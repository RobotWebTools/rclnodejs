`rclnodejs` is a Node.js client for Robot Operating System (ROS) v2.0. It provides extremely simple & easy API for ROS 2.0 programming, the following example shows how to create a ROS 2.0 node and then publish a string message in only 6 lines of code.

``` JavaScript
const rclnodejs = require('rclnodejs');
rclnodejs.init().then(() => {
  const node = rclnodejs.createNode('publisher_example_node');
  const publisher = node.createPublisher('std_msgs/msg/String', 'topic');
  publisher.publish(`Hello ROS 2.0 from rclnodejs`);
  rclnodejs.spin(node);
});
```

# Install ROS 2.0
Befire install rclnodejs, make sure ROS 2.0 is installed first. Read and follow the [Installation Guide](https://github.com/ros2/ros2/wiki/Installation) to install ROS 2.0

# Install rclnodejs
After ROS 2.0 is installed, run the following command

```
npm i --save rclnodejs
```

# More Examples

Use complex message

```JavaScript
  const publisher = node.createPublisher('sensor_msgs/msg/JointState', 'topic');
  publisher.publish({
    header: {
      stamp: {
        sec: 123456,
        nanosec: 789,
      },
      frame_id: 'main frame',
    },
    name: ['Tom', 'Jerry'],
    position: [1, 2],
    velocity: [2, 3],
    effort: [4, 5, 6],
  });

```

Create a subscription

``` JavaScript
const rclnodejs = require('../index.js');

// Create a ROS node and then print out the string message received from publishers
rclnodejs.init().then(() => {
  const node = rclnodejs.createNode('subscription_example_node');

  node.createSubscription('std_msgs/msg/String', 'topic', (msg) => {
    console.log(`Received message: ${typeof msg}`, msg);
  });

  rclnodejs.spin(node);
});
```



Create a service

```JavaScript
  node.createService('example_interfaces/srv/AddTwoInts', 'add_two_ints', (request, response) => {
    console.log(`Incoming request: ${typeof request}`, request);
    let result = response.template;
    result.sum = request.a + request.b;
    console.log(`Sending response: ${typeof result}`, result, '\n--');
    response.send(result);
  });

```

Send a request in a client

```JavaScript
  const client = node.createClient('example_interfaces/srv/AddTwoInts', 'add_two_ints');
  const request = {
    a: Math.floor(Math.random() * 100),
    b: Math.floor(Math.random() * 100),
  };

  console.log(`Sending: ${typeof request}`, request);
  client.sendRequest(request, (response) => {
    console.log(`Result: ${typeof response}`, response);
  });

```

# License
Apache License Version 2.0

