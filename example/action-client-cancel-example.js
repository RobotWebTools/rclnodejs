// Copyright (c) 2020 Matt Richard. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

'use strict';

const rclnodejs = require('../index.js');
const Fibonacci = rclnodejs.require('test_msgs/action/Fibonacci');

class FibonacciActionClient {
  constructor(node) {
    this._node = node;
    this._actionClient = new rclnodejs.ActionClient(
      node,
      'test_msgs/action/Fibonacci',
      'fibonacci'
    );
  }

  async sendGoal(order) {
    await this._actionClient.waitForServer();

    const goal = new Fibonacci.Goal();
    goal.order = order;

    const goalHandle = await this._actionClient.sendGoal(goal, feedback =>
      this.feedbackCallback(feedback)
    );

    this._timer = this._node.createTimer(2000, () =>
      this.timerCallback(goalHandle)
    );

    if (!goalHandle.accepted) {
      console.log('Goal rejected');
      return;
    }

    console.log('Goal accepted');

    const result = await goalHandle.getResult();

    console.log('Result: ', result.result.sequence);

    rclnodejs.shutdown();
  }

  feedbackCallback(feedbackMessage) {
    console.log('Received feedback: ', feedbackMessage.feedback.sequence);
  }

  async timerCallback(goalHandle) {
    this._timer.cancel();

    const response = await goalHandle.cancelGoal();

    if (response.goals_canceling.length > 0) {
      console.log('Goal successfully canceled');
    } else {
      console.log('Goal failed to cancel');
    }
  }
}

rclnodejs
  .init()
  .then(() => {
    const node = rclnodejs.createNode('action_client_example_node');
    const client = new FibonacciActionClient(node);

    client.sendGoal(10);

    rclnodejs.spin(node);
  })
  .catch(err => {
    console.error(err);
  });
