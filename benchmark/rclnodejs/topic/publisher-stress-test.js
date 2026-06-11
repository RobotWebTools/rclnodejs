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

/* eslint-disable camelcase */
import { program } from 'commander';
import rclnodejs from '../../../index.js';

program
  .option('-s, --size <size_kb>', 'The block size', '1')
  .option('-r, --run <n>', 'How many times to run', '1')
  .parse(process.argv);

const options = program.opts();
const size = parseInt(options.size) || 1;
const totalTimes = parseInt(options.run) || 1;

async function main() {
  try {
    await rclnodejs.init();

    const startTime = process.hrtime.bigint();
    const node = rclnodejs.createNode('stress_publisher_rclnodejs');
    const publisher = node.createPublisher(
      'std_msgs/msg/UInt8MultiArray',
      'stress_topic'
    );

    let sentTimes = 0;

    const message = {
      layout: {
        dim: [
          { label: 'height', size: 10, stride: 600 },
          { label: 'width', size: 20, stride: 60 },
          { label: 'channel', size: 3, stride: 4 },
        ],
        data_offset: 0,
      },
      data: new Uint8Array(1024 * size).map((_, index) => index & 0xff),
    };

    node
      .getLogger()
      .info(
        `The publisher will publish a UInt8MultiArray topic (contains a size of ${size}KB array) ${totalTimes} times.`
      );

    // Publish messages in a controlled manner
    const publishMessages = () => {
      return new Promise((resolve) => {
        const publishNext = () => {
          if (sentTimes < totalTimes) {
            publisher.publish(message);
            sentTimes++;
            setImmediate(publishNext);
          } else {
            resolve();
          }
        };
        publishNext();
      });
    };

    await publishMessages();

    const endTime = process.hrtime.bigint();
    const diffNanos = endTime - startTime;
    const diffMillis = Number(diffNanos) / 1000000;
    const seconds = Math.floor(diffMillis / 1000);
    const milliseconds = Math.round(diffMillis % 1000);

    node
      .getLogger()
      .info(
        `Benchmark took ${seconds} seconds and ${milliseconds} milliseconds.`
      );

    await rclnodejs.shutdown();
  } catch (error) {
    console.error('Error in publisher stress test:', error);
    process.exit(1);
  }
}

main();
