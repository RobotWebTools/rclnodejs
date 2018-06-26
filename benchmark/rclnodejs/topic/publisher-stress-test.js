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
const rclnodejs = require('../../../index.js');

app
  .option('-s, --size [size_kb]', 'The block size')
  .option('-r, --run <n>', 'How many times to run')
  .parse(process.argv);

rclnodejs.init().then(() => {
  const time = process.hrtime();
  let node = rclnodejs.createNode('stress_publisher_rclnodejs');
  const publisher = node.createPublisher('std_msgs/msg/UInt8MultiArray', 'stress_topic');
  let sentTimes = 0;
  let totalTimes = app.run || 1;
  let size = app.size || 1;
  const message = {
    layout: {
      dim: [
        {label: 'height',  size: 10, stride: 600},
        {label: 'width',   size: 20, stride: 60},
        {label: 'channel', size: 3,  stride: 4},
      ],
      data_offset: 0,
    },
    data: Uint8Array.from({length: 1024 * size}, (v, k) => k)
  };
  console.log(`The publisher will publish a UInt8MultiArray topic(contains a size of ${size}KB array)` +
  ` ${totalTimes} times.`);

  setImmediate(() => {
    while (sentTimes++ < totalTimes) {
      publisher.publish(message);
    }
    rclnodejs.shutdown();
    const diff = process.hrtime(time);
    console.log(`Benchmark took ${diff[0]} seconds and ${Math.ceil(diff[1] / 1000000)} milliseconds.`);
  });

  rclnodejs.spin(node);
}).catch((err) => {
  console.log(err);
});
