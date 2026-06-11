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

import rclnodejs from '../../../index.js';

async function main() {
  try {
    await rclnodejs.init();

    const node = rclnodejs.createNode('stress_subscription_rclnodejs');

    node.createSubscription(
      'std_msgs/msg/UInt8MultiArray',
      'stress_topic',
      (message) => {
        // Just consume the message for benchmarking
        // In a real scenario, you might process the message here
      }
    );

    node.getLogger().info('Subscription node ready to receive messages');

    // Set up graceful shutdown
    process.on('SIGINT', async () => {
      node.getLogger().info('Shutting down subscription node...');
      await rclnodejs.shutdown();
      process.exit(0);
    });

    await rclnodejs.spin(node);
  } catch (error) {
    console.error('Error in subscription stress test:', error);
    process.exit(1);
  }
}

main();
