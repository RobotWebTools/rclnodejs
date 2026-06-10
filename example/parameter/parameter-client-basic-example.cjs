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

const rclnodejs = require('../../index.js').default;

async function main() {
  await rclnodejs.init();

  const node = rclnodejs.createNode('param_client_node');

  const paramClient = node.createParameterClient('turtlesim', {
    timeout: 5000,
  });

  try {
    const available = await paramClient.waitForService(10000);

    if (!available) {
      console.log('Turtlesim node not available. Please run:');
      console.log('  ros2 run turtlesim turtlesim_node');
      return;
    }

    const param = await paramClient.getParameter('background_b');
    console.log(`Current background_b: ${param.value}`);

    await paramClient.setParameter('background_b', 200);
    const updated = await paramClient.getParameter('background_b');
    console.log(`Updated background_b: ${updated.value}`);
  } catch (error) {
    console.error('Error:', error.message);
  } finally {
    node.destroy();
    rclnodejs.shutdown();
  }
}

main();
