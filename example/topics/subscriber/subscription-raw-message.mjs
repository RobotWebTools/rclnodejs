// Copyright (c) 2020 Intel Corporation. All rights reserved.
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

// From an installed package you would write `import rclnodejs from 'rclnodejs'`;
// run from this checkout we import the source entry point directly.

import rclnodejs from '../../../index.js';

await rclnodejs.init();

const node = rclnodejs.createNode('subscription_message_example_node');

node.createSubscription(
  'test_msgs/msg/BasicTypes',
  'chatter',
  { isRaw: true },
  (msg) => {
    console.log(`Received message: ${msg.toString('utf8')}`);
  }
);

rclnodejs.spin(node);
