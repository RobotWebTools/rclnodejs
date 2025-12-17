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
const { throttleTime, map, filter, bufferCount } = require('rxjs');

async function main() {
  await rclnodejs.init();

  const node = rclnodejs.createNode('observable_subscription_example_node');

  // Basic observable subscription
  const obsSub = node.createObservableSubscription(
    'std_msgs/msg/String',
    'topic'
  );

  // Example 1: Throttled messages (max 2/sec)
  obsSub.observable
    .pipe(
      throttleTime(500),
      map((msg) => msg.data)
    )
    .subscribe((data) => {
      console.log('[Throttled]', data);
    });

  // Example 2: Filtered messages (only containing "ROS")
  obsSub.observable
    .pipe(
      map((msg) => msg.data),
      filter((data) => data.includes('ROS'))
    )
    .subscribe((data) => {
      console.log('[Filtered]', data);
    });

  // Example 3: Batched messages (every 3 messages)
  obsSub.observable
    .pipe(
      map((msg) => msg.data),
      bufferCount(3)
    )
    .subscribe((batch) => {
      console.log('[Batched]', batch.length, 'messages');
    });

  console.log('Observable subscription created on /topic');
  console.log(
    'Run: ros2 topic pub /topic std_msgs/msg/String "{data: Hello ROS}" -r 5'
  );

  rclnodejs.spin(node);
}

main();
