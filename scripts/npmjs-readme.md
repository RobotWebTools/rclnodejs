# rclnodejs [![Build Status](https://travis-ci.org/RobotWebTools/rclnodejs.svg?branch=develop)](https://travis-ci.org/RobotWebTools/rclnodejs)

`rclnodejs` is a Node.js client for the Robot Operating System (ROS) v2.0. It provides a simple and easy JavaScript API for ROS 2.0 programming. TypeScript declarations are included to support use of rclnodejs in TypeScript projects. 

The following JavaScript example demonstrates how to create a ROS 2.0 node and then publish a string message in only 6 lines of code. 

``` JavaScript
const rclnodejs = require('rclnodejs');
rclnodejs.init().then(() => {
  const node = rclnodejs.createNode('publisher_example_node');
  const publisher = node.createPublisher('std_msgs/msg/String', 'topic');
  publisher.publish(`Hello ROS 2.0 from rclnodejs`);
  rclnodejs.spin(node);
});
```

## Install ROS 2.0

Before install rclnodejs, make sure ROS 2.0 is installed first. Read and follow the [Installation Guide](https://index.ros.org/doc/ros2/Installation/) to install ROS 2.0

## Install rclnodejs

After ROS 2.0 is installed, run the following command

``` bash
npm i --save rclnodejs
```

## Match with ROS 2.0 Stable Releases

Please check the [table](https://github.com/RobotWebTools/rclnodejs#match-with-ros-20-stable-releases) to select a specific version you want.

## Document

API [documentation](http://robotwebtools.org/rclnodejs/docs/index.html) is available online.

## JavaScript Examples

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

## Using rclnodejs with TypeScript
In your node project install the rclnodejs package as described above. You will also need the TypeScript compiler and node typings installed.
```
  npm install typescript @types/node -D
```

In your tsconfig.json file include the following compiler options:
```json
{
  "compilerOptions": {
    "module": "commonjs",
    "moduleResolution": "node",
    "target": "es6",
    ...
  }
}
```

Here's an earlier JavaScript example reimplemented in TypeScript.
```
import * as rclnodejs from 'rclnodejs';
rclnodejs.init().then(() => {
  const node = rclnodejs.createNode('publisher_example_node');
  const publisher = node.createPublisher('std_msgs/msg/String', 'topic');
  publisher.publish(`Hello ROS 2.0 from rclnodejs`);
  rclnodejs.spin(node);
});
```

In this example you may notice that other than the `import` statement the code is virtually identical to the JavaScript version. Where the benefits of using TypeScript kick in is when using smart coding tools such as Visual Studio Code or the CodeMix plugin for Eclipse. These productivity tools use the rclnodejs type declaration files to help you learn the api and code with it more accurately and quickly.

Type aliases are provided for the ROS2 messages in the types/interfaces.d.ts file. To use a message type alias follow the naming pattern <pkg_name>.[msg|srv].<type>, e.g., sensor_msgs.msg.LaserScan or the std_msgs.msg.String as shown below. 
```
   const msg: rclnodejs.std_msgs.msg.String = {
     data: 'hello ROS2 from rclnodejs'
   }
```
**Note** that the interface.d.ts file is updated each time the generate_messages.js script is run.

While the code snippet above is a trivial message example, when working with more complex message types such as sensor data, developers can benefit from typing information to help detect issues earlier in the develpment cycle.

## License

Apache License Version 2.0
