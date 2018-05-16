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

rl.question('The amount of data(KB) to be sent for each request. ', (amount) => {
  amount = parseInt(amount, 10);
  const mapData = {
    map: {
      header: {
        stamp: {
          sec: 123456,
          nanosec: 789,
        },
        frame_id: 'main_frame'
      },
      info: {
        map_load_time: {
          sec: 123456,
          nanosec: 789,
        },
        resolution: 1.0,
        width: 1024,
        height: 768,
        origin: {
          position: {
            x: 0.0,
            y: 0.0,
            z: 0.0
          },
          orientation: {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 0.0
          }
        }
      },
      data: Int8Array.from({length: 1024 * amount}, (v, k) => k)
    }
  };

  rclnodejs.init().then(() => {
    let node = rclnodejs.createNode('stress_service_rclnodejs');
    node.createService('nav_msgs/srv/GetMap', 'get_map', (request, response) => {
      return mapData;
    });
    rclnodejs.spin(node);
  }).catch((e) => {
    console.log(`Error: ${e}`);
  });

  rl.close();
});
