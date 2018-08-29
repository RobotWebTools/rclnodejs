# Introduction
Actionlib tutorial based on `rclnodejs`.

## Actionlib background
To continue the tutorial, you should have some basic understanding of the `action` term in ROS. If you're not familar with it, please refer to the [description](http://wiki.ros.org/actionlib/DetailedDescription).

## Precondition
You should prepare an action text file. The content is like this:

```
# Define the goal
uint32 dishwasher_id  # Specify which dishwasher we want to use
---
# Define the result
uint32 total_dishes_cleaned
---
# Define a feedback message
float32 percent_complete
# Use a message from another package, to prove that it works
sensor_msgs/Image image
```

Fortunately, there is a `dodishes.action` file that is already available in the `test/ros1_actions` directory.

## Basic steps of processing action files for rclnodejs
For `rclnodejs`, the basic steps of process an action file is separated into 2 steps:
* Generate several msg files and the js message package from the action file.
* Build the shared library from the msg files that will be used in running actionlib-feature-related code.

### Generate msg files from the action file
The `generated` directory contains the generated js message package for `rclnodejs`. If you have run `npm install`, then it should
exist after `npm install` was done. However, since your `AMENT_PREFIX_PATH` may not include the directory path of the action file, the `ros1_actions` package may not be generated. So you need remove the whole `generated` directory and regenerate them.
```
$ rm -fr generated
$ export AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH:$(pwd)/test/ros1_actions
$ node scripts/generate_messages.js
```

### Build the shared library from the msg files
```
$ cd test/ros1_actions
$ colcon build
```

## Run the action example
After the build, you can run the action example. The action example contains two parts, one is the [action server](./example/action-server-example.js), another is the [action client](./example/action-client-example.js). When running the example, you should launch the server first, and then launch the client.

* Launch a terminal session, load ROS2 environment and go to `rclnodejs` directory.
```
$ source test/ros1_actions/install/local_setup.bash
$ node example/action-server-example.js
```

* Launch another terminal session, load ROS2 environment and go to `rclnodejs` directory.
```
$ source test/ros1_actions/install/local_setup.bash
$ node example/action-client-example.js
```

Here is the action client output:
```
The goal was sent, the goal id is 7c28f24a-5ce8-4b13-a2aa-7aa62128fd03
70% of the task has been completed.
The goal, whose id is 7c28f24a-5ce8-4b13-a2aa-7aa62128fd03, has been executed successfully.
10 dishes have been cleaned.
The goal, whose id is 7c28f24a-5ce8-4b13-a2aa-7aa62128fd03, has been executed successfully.
```

And here is the action server output:
```
A goal, whose id is 7c28f24a-5ce8-4b13-a2aa-7aa62128fd03, was received.
```

## Explaination of the action example:
1. Action server
For the action server, the skeleton code is like this:
```
rclnodejs.init().then(() => {
  const as = new rclnodejs.ActionLib.ActionServer({
    type: 'ros1_actions/msg/DoDishes',
    actionServer: 'dishes',
    rclnodejs: rclnodejs
  });

  as.on('goal', function(goal) {
    goal.setAccepted('goal accepted');
    goal.publishFeedback(feedback);
    setTimeout(() => {
      goal.setSucceeded({total_dishes_cleaned: 10}, 'done');
    }, 500);
  });

  as.on('cancel', (goalHandle) => {
    // cancel handler code
  });

  as.start();
}).catch((err) => {
  console.error(err);
});
```

First, you should new an `ActionServer` with the `type` and the `actionServer`. The `type` is the action package name and the `actionServer` is the name of the action server, which should be the same as the action client request to in future.

The `ActionServer` can emit 2 type events: `goal` and `cancel` events.
* `goal` event: triggered when an action client sends an goal to the action server by calling its `sendGoal()` method. In the handler of the this event, you can accept the goal by calling `goal.setAccepted()`. During executing the goal, you can send a feedback to the action client by calling `goal.publishFeedback()`. Once the goal is completed, you should set the goal status by calling `goal.setSucceeded()`, which will trigger the `result` event for the action client.
* `cancel` event: triggered when an action client cancels the the goal after a goal was sent to the action server but not completed yet.

The `start()` method must be called to start the action server.

2. Action client
For the action client, the skeleton code is like this:
```
rclnodejs.init().then(() => {
  const GoalStatus = rclnodejs.require('actionlib_msgs/msg/GoalStatus');

  const ac = new rclnodejs.ActionClientInterface({
    type: 'ros1_actions/msg/DoDishes',
    actionServer: 'dishes',
    rclnodejs: rclnodejs
  });

  let goal = ac.sendGoal({ goal: {dishwasher_id: 1}});

  ac.on('feedback', (feedback) => {
    // feedback handler
  });

  ac.on('status', (status) => {
    status.status_list.forEach((s) =>{
      if (s.goal_id.id === goal.goal_id.id &&
          s.status === GoalStatus.SUCCEEDED) {
        console.log(`The goal, whose id is ${s.goal_id.id}, has been executed successfully.`);
      }
    });
  });

  ac.on('result', (result) => {
    // result handler
  });
}).catch((err) => {
  console.error(err);
});
```

To construct an action client, use `new rclnodejs.ActionClientInterface()`. The action client can emit 3 type events:
* `status` event: tirggered when the action server sends messages to the action client during the goal is executing.
* `feedback` event: triggered after the action server called `publishFeedback`.
* `result` event: triggered after the action server called `setSucceeded`.

**Notice**, the action state transitions must obey some specific order, for more details please refer to [this articile](http://wiki.ros.org/actionlib/DetailedDescription)
