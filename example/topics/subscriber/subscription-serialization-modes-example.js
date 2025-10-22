// Copyright (c) 2024 rclnodejs contributors. All rights reserved.
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
 * This example demonstrates the use of serialization modes for subscriptions.
 * Serialization modes allow you to control how TypedArrays are handled in messages:
 * - 'typed' (default): Keep TypedArrays for performance
 * - 'plain': Convert TypedArrays to regular arrays
 * - 'json': Fully JSON-safe (converts TypedArrays, BigInt, Infinity, etc.)
 */
async function main() {
  await rclnodejs.init();
  const node = new rclnodejs.Node('serialization_modes_example_node');

  // Default mode: 'typed' - keeps TypedArrays
  node.createSubscription(
    'sensor_msgs/msg/LaserScan',
    '/laser_scan',
    { serializationMode: 'typed' },
    (msg) => {
      console.log(
        `[TYPED] ranges: ${msg.ranges ? msg.ranges.constructor.name : 'undefined'}`
      );
    }
  );

  // Plain mode: converts TypedArrays to regular arrays
  node.createSubscription(
    'sensor_msgs/msg/LaserScan',
    '/laser_scan',
    { serializationMode: 'plain' },
    (msg) => {
      console.log(
        `[PLAIN] ranges: ${msg.ranges ? msg.ranges.constructor.name : 'undefined'}`
      );
    }
  );

  // JSON mode: fully JSON-safe
  node.createSubscription(
    'sensor_msgs/msg/LaserScan',
    '/laser_scan',
    { serializationMode: 'json' },
    (msg) => {
      console.log(
        `[JSON] ranges: ${msg.ranges ? msg.ranges.constructor.name : 'undefined'}, JSON-safe: ${canStringifyJSON(msg)}`
      );
    }
  );

  node.spin();
}

function canStringifyJSON(obj) {
  try {
    JSON.stringify(obj);
    return true;
  } catch {
    return false;
  }
}

main().catch(console.error);
