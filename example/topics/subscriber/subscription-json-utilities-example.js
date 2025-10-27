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

const rclnodejs = require('../../../index.js');

/**
 * This example demonstrates the JSON utility functions for manual message conversion.
 * These utilities are useful when you need to convert messages on-demand
 * rather than using the serializationMode subscription option.
 */
async function main() {
  await rclnodejs.init();
  const node = new rclnodejs.Node('json_utilities_example_node');

  node.createSubscription('sensor_msgs/msg/LaserScan', '/laser_scan', (msg) => {
    // Convert using utility functions
    const jsonSafe = rclnodejs.toJSONSafe(msg);
    const jsonString = rclnodejs.toJSONString(msg);

    console.log(
      `Original: ${msg.ranges ? msg.ranges.constructor.name : 'undefined'}, JSON-safe: ${jsonSafe.ranges ? jsonSafe.ranges.constructor.name : 'undefined'}, JSON length: ${jsonString.length}`
    );
  });

  node.spin();
}

main().catch(console.error);
