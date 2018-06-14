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
const app = require('commander');
const rclnodejs = require('../../index.js');

app
  .option('-p, --period [period_ms]', 'The period(ms) to send a topic')
  .option('-r, --run <n>', 'How many times to run')
  .parse(process.argv);

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

  const time = process.hrtime();
  let node = rclnodejs.createNode('endurance_publisher_rclnodejs');
  let publisher = node.createPublisher(JointState, 'endurance_topic');
  let sentTimes = 0;
  let totalTimes = app.run || 1;
  let period = app.period || 1;
  console.log(`The publisher will publish a JointState topic ${totalTimes} times every ${period}ms.`);

  let timer = setInterval(() => {
    if (sentTimes++ > totalTimes) {
      rclnodejs.shutdown();
      clearInterval(timer);
      const diff = process.hrtime(time);
      console.log(`Benchmark took ${diff[0]} seconds and ${Math.ceil(diff[1] / 1000000)} milliseconds.`);
    } else {
      publisher.publish(state);
    }}, period);
  rclnodejs.spin(node);
}).catch((err) => {
  console.log(err);
});
