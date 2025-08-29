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
const { program } = require('commander');
const rclnodejs = require('../../../index.js');

program
  .option('-r, --run <n>', 'How many times to run', '1')
  .parse(process.argv);

const options = program.opts();
const totalTimes = parseInt(options.run) || 1;

async function main() {
  try {
    await rclnodejs.init();

    const startTime = process.hrtime.bigint();
    const node = rclnodejs.createNode('stress_client_rclnodejs');
    const client = node.createClient('nav_msgs/srv/GetMap', 'get_map', {
      enableTypedArray: true,
    });

    let receivedTimes = 0;

    node
      .getLogger()
      .info(
        `The client will send a GetMap request continuously until receiving response ${totalTimes} times.`
      );

    // Wait for service to be available - simplified approach
    node.getLogger().info('Waiting for service to be available...');

    // Simple wait with spinning
    await new Promise((resolve) => {
      setTimeout(() => {
        node.getLogger().info('Service should be available now');
        resolve();
      }, 2000); // Wait 2 seconds for service to start
    });

    // Send requests sequentially using callback pattern
    const sendRequests = async () => {
      // Start spinning to handle callbacks
      rclnodejs.spin(node);

      for (let i = 0; i < totalTimes; i++) {
        try {
          const response = await new Promise((resolve, reject) => {
            client.sendRequest({}, (response) => {
              console.log('send+++');
              console.log(response);
              if (response) {
                resolve(response.map.header);
              } else {
                reject(new Error('No response received'));
              }
            });
          });
          if (response) {
            receivedTimes++;
            // Show progress every 100 requests
            if (receivedTimes % 100 === 0) {
              node
                .getLogger()
                .info(
                  `Progress: ${receivedTimes}/${totalTimes} requests completed`
                );
            }
          }
        } catch (error) {
          node.getLogger().error(`Request ${i + 1} failed: ${error.message}`);
        }
      }
    };

    await sendRequests();

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
    node
      .getLogger()
      .info(`Successfully received ${receivedTimes}/${totalTimes} responses`);

    await rclnodejs.shutdown();
  } catch (error) {
    console.error('Error in client stress test:', error);
    process.exit(1);
  }
}

main();
