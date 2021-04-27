# rclnodejs [![Build Status](https://travis-ci.org/RobotWebTools/rclnodejs.svg?branch=develop)](https://travis-ci.org/RobotWebTools/rclnodejs)

`rclnodejs` is a Node.js client for the Robot Operating System (ROS 2). It provides a simple and easy JavaScript API for ROS 2 programming. TypeScript declarations are included to support use of rclnodejs in TypeScript projects.

Here's an example for how to create a ROS 2 node that publishes a string message in a few lines of JavaScript.

```JavaScript
const rclnodejs = require('rclnodejs');
rclnodejs.init().then(() => {
  const node = rclnodejs.createNode('publisher_example_node');
  const publisher = node.createPublisher('std_msgs/msg/String', 'topic');
  publisher.publish(`Hello ROS 2 from rclnodejs`);
  rclnodejs.spin(node);
});
```

## Prerequisites

**Node.js**

- [Node.js](https://nodejs.org/en/) version between 8.12 - 12.x.

**ROS 2 SDK**

- See the ROS 2 SDK [Installation Guide](https://index.ros.org/doc/ros2/Installation/) for details.
- **DON'T FORGET TO [SOURCE THE ROS 2 STARTUP FILES](https://index.ros.org/doc/ros2/Tutorials/Configuring-ROS2-Environment/#source-the-setup-files)**

## Install rclnodejs

Install the rclnodejs version that is compatible with your installed version of ROS 2 (see table below).

Run the following command for the most current version of rclnodejs

```bash
npm i rclnodejs
```

or to install a specific version of rclnodejs use

```bash
npm i rclnodejs@x.y.z
```

#### RCLNODEJS - ROS 2 Version Compatibility

|                                                            RCLNODEJS Version                                                            |                                                                         Compatible ROS 2 Release                                                                         |
| :-------------------------------------------------------------------------------------------------------------------------------------: | :----------------------------------------------------------------------------------------------------------------------------------------------------------------------: |
| [0.18.2 (current)](https://www.npmjs.com/package/rclnodejs/v/0.18.2) ([API](http://robotwebtools.org/rclnodejs/docs/0.18.0/index.html)) | [Foxy Fitzroy](https://github.com/ros2/ros2/releases/tag/release-foxy-20201211) / [Eloquent Elusor](https://github.com/ros2/ros2/releases/tag/release-eloquent-20200124) |
|                                [0.10.3](https://github.com/RobotWebTools/rclnodejs/releases/tag/0.10.3)                                 |                                    [Dashing Diademata - Patch 4](https://github.com/ros2/ros2/releases/tag/release-dashing-20191018)                                     |

## Documentation

API [documentation](http://robotwebtools.org/rclnodejs/docs/index.html) is available online.

## JavaScript Examples

The source for the following examples and many others can be found [here](https://github.com/RobotWebTools/rclnodejs/tree/develop/example).

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

```JavaScript
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

In your project's tsconfig.json file include the following compiler options:

```jsonc
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

```typescript
import * as rclnodejs from 'rclnodejs';
rclnodejs.init().then(() => {
  const node = rclnodejs.createNode('publisher_example_node');
  const publisher = node.createPublisher('std_msgs/msg/String', 'topic');
  publisher.publish(`Hello ROS 2 from rclnodejs`);
  rclnodejs.spin(node);
});
```

Type-aliases for the ROS2 messages can be found in the `types/interfaces.d.ts` file. To use a message type-alias follow the naming pattern <pkg_name>.[msg|srv].<type>, e.g., sensor_msgs.msg.LaserScan or the std_msgs.msg.String as shown below.

```typescript
const msg: rclnodejs.std_msgs.msg.String = {
  data: 'hello ROS2 from rclnodejs',
};
```

**Note** that the interface.d.ts file is updated each time the generate_messages.js script is run.

## License

Apache License Version 2.0
