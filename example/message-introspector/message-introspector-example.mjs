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

// From an installed package you would write `import rclnodejs from 'rclnodejs'`;
// run from this checkout we import the source entry point directly.

import rclnodejs from '../../index.js';

/**
 * This example demonstrates the MessageIntrospector class for
 * inspecting ROS 2 message structure without using loader.loadInterface.
 */
await rclnodejs.init();

const Twist = new rclnodejs.MessageIntrospector('geometry_msgs/msg/Twist');
const String = new rclnodejs.MessageIntrospector('std_msgs/msg/String');
const JointState = new rclnodejs.MessageIntrospector(
  'sensor_msgs/msg/JointState'
);

console.log('Twist fields:', Twist.fields);
console.log('Twist defaults:', Twist.defaults);

console.log('String fields:', String.fields);
console.log('String defaults:', String.defaults);

console.log('JointState fields:', JointState.fields);

console.log('Twist schema msgName:', Twist.schema.msgName);

await rclnodejs.shutdown();
