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

  async sendGoal() {
    this._node.getLogger().info('Waiting for action server...');
    await this._actionClient.waitForServer();

    const goal = new Fibonacci.Goal();
    goal.order = 10;

    this._node.getLogger().info('Sending goal request...');

    const goalHandle = await this._actionClient.sendGoal(goal, (feedback) =>
      this.feedbackCallback(feedback)
    );

    if (!goalHandle.accepted) {
      this._node.getLogger().info('Goal rejected');
      return;
    }

    this._node.getLogger().info('Goal accepted');

    // Start a 2 second timer
    this._timer = this._node.createTimer(2000, () =>
      this.timerCallback(goalHandle)
    );
  }

  feedbackCallback(feedbackMessage) {
    this._node
      .getLogger()
      .info(`Received feedback: ${feedbackMessage.feedback.sequence}`);
  }

  async timerCallback(goalHandle) {
    this._node.getLogger().info('Canceling goal');
    // Cancel the timer
    this._timer.cancel();

    const response = await goalHandle.cancelGoal();

    if (response.goals_canceling.length > 0) {
      this._node.getLogger().info('Goal successfully canceled');
    } else {
      this._node.getLogger().info('Goal failed to cancel');
    }

    rclnodejs.shutdown();
  }
}

rclnodejs
  .init()
  .then(() => {
    const node = rclnodejs.createNode('action_client_example_node');
    const client = new FibonacciActionClient(node);

    client.sendGoal();

    rclnodejs.spin(node);
  })
  .catch((err) => {
    console.error(err);
  });
