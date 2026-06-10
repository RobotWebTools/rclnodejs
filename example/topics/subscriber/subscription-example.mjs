// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
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

// ESM variant of subscription-example.js.
//
// From an installed package you would write `import rclnodejs from 'rclnodejs'`;
// run from this checkout we import the source entry point directly.

import rclnodejs from '../../../index.js';

await rclnodejs.init();

const node = rclnodejs.createNode('subscription_example_node');

node.createSubscription('std_msgs/msg/String', 'topic', (msg) => {
  console.log(`Received message: ${typeof msg}`, msg);
});

rclnodejs.spin(node);
