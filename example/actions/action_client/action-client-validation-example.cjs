// Copyright (c) 2025 Mahmoud Alghalayini. All rights reserved.
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

const rclnodejs = require('../../../index.js').default;

async function main() {
  await rclnodejs.init();
  const node = rclnodejs.createNode('action_client_validation_example_node');

  const Fibonacci = rclnodejs.require(
    'action_tutorials_interfaces/action/Fibonacci'
  );

  const actionClient = new rclnodejs.ActionClient(
    node,
    'action_tutorials_interfaces/action/Fibonacci',
    'fibonacci',
    { validateGoals: true }
  );

  const validGoal = { order: 10 };
  console.log(
    'Valid goal:',
    rclnodejs.validateMessage(validGoal, Fibonacci.Goal).valid
  );

  try {
    await actionClient.sendGoal({ order: 'invalid' });
  } catch (error) {
    if (error instanceof rclnodejs.MessageValidationError) {
      console.log('Caught validation error:', error.issues[0].problem);
    }
  }

  actionClient.destroy();
  node.destroy();
  rclnodejs.shutdown();
}

main();
