// Copyright (c) 2018 Intel Corporation. All rights reserved.
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

class FibonacciActionServer {
  constructor(node) {
    this._node = node;

    this._actionServer = new rclnodejs.ActionServer(
      node,
      'test_msgs/action/Fibonacci',
      'fibonacci',
      this.executeCallback.bind(this),
      this.goalCallback.bind(this),
      null,
      this.cancelCallback.bind(this)
    );
  }

  async executeCallback(goalHandle) {
    this._node.getLogger().info('Executing goal...');

    const feedbackMessage = new Fibonacci.Feedback();
    const sequence = [0, 1];

    // Start executing the action
    for (let i = 1; i < goalHandle.request.order; i++) {
      // Check if the goal has been canceled
      if (goalHandle.isCancelRequested) {
        goalHandle.canceled();
        this._node.getLogger().info('Goal canceled');
        return new Fibonacci.Result();
      }

      // Update Fibonacci sequence
      sequence.push(sequence[i] + sequence[i - 1]);

      feedbackMessage.sequence = sequence;
      this._node
        .getLogger()
        .info(`Publishing feedback: ${feedbackMessage.sequence}`);

      // Publish the feedback
      goalHandle.publishFeedback(feedbackMessage);

      // Wait for 1 second
      await new Promise(resolve => setTimeout(resolve, 1000));
    }

    goalHandle.succeed();

    // Populate result message
    const result = new Fibonacci.Result();
    result.sequence = sequence;

    this._node.getLogger().info(`Returning result: ${result.sequence}`);

    return result;
  }

  goalCallback(goalHandle) {
    this._node.getLogger().info('Received goal request');
    return rclnodejs.GoalResponse.ACCEPT;
  }

  cancelCallback(goalHandle) {
    this._node.getLogger().info('Received cancel request');
    return rclnodejs.CancelResponse.ACCEPT;
  }
}

rclnodejs
  .init()
  .then(() => {
    const node = rclnodejs.createNode('action_server_example_node');

    new FibonacciActionServer(node);

    rclnodejs.spin(node);
  })
  .catch(err => {
    console.error(err);
  });
