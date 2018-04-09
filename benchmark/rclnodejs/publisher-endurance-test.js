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

'use strict';

/* eslint-disable camelcase */
const rclnodejs = require('../../index.js');
const readline = require('readline');

const rl = readline.createInterface({
  input: process.stdin,
  output: process.stdout
});

rl.question('How many times do you want to run? ', (times) => {
  rl.question('Please enter the period of publishing a topic in millisecond ', (ms) => {
    rclnodejs.init().then(() => {
      const JointState = 'sensor_msgs/msg/JointState';
      const state = {
        header: {
          stamp: {
            sec: 123456,
            nanosec: 789,
          },
          frame_id: 'main_frame',
        },
        name: ['Tom', 'Jerry'],
        position: [1, 2],
        velocity: [2, 3],
        effort: [4, 5, 6],
      };
      let period = parseInt(ms, 10);

      console.log(`The publisher will publish a JointState topic ${times} times every ${period}ms.`);
      console.log(`Begin at ${new Date().toString()}.`);

      let node = rclnodejs.createNode('endurance_publisher_rclnodejs');
      let publisher = node.createPublisher(JointState, 'endurance_topic');
      let sentTimes = 0;
      let totalTimes = parseInt(times, 10);

      let timer = setInterval(() => {
        if (sentTimes++ > totalTimes) {
          clearInterval(timer);
          rclnodejs.shutdown();
          console.log(`End at ${new Date().toString()}`);
        } else {
          publisher.publish(state);
        }}, period);
      rclnodejs.spin(node);
    }).catch((err) => {
      console.log(err);
    });

    rl.close();
  });
});
