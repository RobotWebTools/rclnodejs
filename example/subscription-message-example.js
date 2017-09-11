// Copyright (c) 2017 Intel Corporation. All rights reserved.
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
let JointState = rclnodejs.require('sensor_msgs').msg.JointState;

rclnodejs.init().then(() => {
  let node = rclnodejs.createNode('subscription_example_node');
  let count = 0;

  node.createSubscription(JointState, 'JointState', (state) => {
    console.log(`Received ${++count} messages:`);
    console.log('state.header.stamp.sec = ' + state.header.stamp.sec);
    console.log('state.header.stamp.nanosec = ' + state.header.stamp.nanosec);
    console.log('state.header.frame_id = ' + state.header.frame_id);
    console.log('state.name = ' + state.name.toString());
    console.log('state.position = ' + state.position.toString());
    console.log('state.velocity = ' + state.velocity.toString());
    console.log('state.effort = ' + state.effort.toString() + '\n');
  });

  rclnodejs.spin(node);
}).catch(e => {
  console.log(e);
});
