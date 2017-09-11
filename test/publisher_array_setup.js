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
let Header = rclnodejs.require('std_msgs').msg.Header;
let Time = rclnodejs.require('builtin_interfaces').msg.Time;
let JointState = rclnodejs.require('sensor_msgs').msg.JointState;

rclnodejs.init().then(function() {
  const node = rclnodejs.createNode('array_message_publisher');
  const publisher = node.createPublisher(JointState, 'JointState');

  let time = new Time();
  time.sec = 123456;
  time.nanosec = 789;

  let header = new Header();
  header.stamp = time;
  // eslint-disable-next-line
  header.frame_id = 'main frame';

  let state = new JointState();
  state.header = header;
  state.name = ['Tom', 'Jerry'];
  state.position = [1, 2];
  state.velocity = [2, 3];
  state.effort = [4, 5, 6];

  publisher.publish(state);
  rclnodejs.spin(node);

  setTimeout(() => {
    node.destroy();
    rclnodejs.shutdown();
  }, 100);
}).catch(function(err) {
  console.log(err);
});
