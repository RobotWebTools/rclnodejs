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
  .option('-s, --size <size_kb>', 'The block size in KB', '1')
  .parse(process.argv);

const options = program.opts();
let sizeKB = parseInt(options.size) || 1;

// Calculate map dimensions based on size
// Each cell is 1 byte (int8), so for sizeKB kilobytes we need sizeKB * 1024 cells
const totalCells = sizeKB * 1024;

// Calculate width and height as close to square as possible
const width = Math.floor(Math.sqrt(totalCells));
const height = Math.ceil(totalCells / width);
const actualCells = width * height;

console.log(`Requested size: ${sizeKB}KB (${totalCells} cells)`);
console.log(
  `Calculated dimensions: ${width} x ${height} = ${actualCells} cells`
);
console.log(`Actual size: ${(actualCells / 1024).toFixed(2)}KB`);

rclnodejs
  .init()
  .then(() => {
    let node = rclnodejs.createNode('stress_service_rclnodejs');

    node.createService('nav_msgs/srv/GetMap', 'get_map', () => {
      // Create map data structure with calculated dimensions
      const mapData = {
        map: {
          header: {
            stamp: {
              sec: 123456,
              nanosec: 789,
            },
            frame_id: 'main_frame',
          },
          info: {
            map_load_time: {
              sec: 123456,
              nanosec: 789,
            },
            resolution: 1.0,
            width: width,
            height: height,
            origin: {
              position: {
                x: 0.0,
                y: 0.0,
                z: 0.0,
              },
              orientation: {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 0.0,
              },
            },
          },
          // Generate data array with the exact size needed (width * height)
          data: new Int8Array(actualCells).fill(0),
        },
      };

      return mapData;
    });

    console.log(`GetMap service started`);
    console.log(`Map dimensions: ${width} x ${height} = ${actualCells} cells`);
    console.log(`Data size: ${(actualCells / 1024).toFixed(2)}KB`);
    rclnodejs.spin(node);
  })
  .catch((e) => {
    console.log(`Error: ${e}`);
  });
