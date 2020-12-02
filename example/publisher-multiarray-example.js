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

/* eslint-disable camelcase */

const rclnodejs = require('../index.js');

rclnodejs
  .init()
  .then(() => {
    const node = rclnodejs.createNode('publisher_multiarray_example_node');
    const publisher = node.createPublisher(
      'std_msgs/msg/Int32MultiArray',
      'Int32MultiArray'
    );

    let count = 0;
    setInterval(function() {
      // Please reference the usage of multi-array at
      // https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/MultiArrayLayout.msg
      publisher.publish({
        layout: {
          dim: [
            {
              label: 'height',
              size: 2,
              stride: 2 * 3 * 3,
            },
            {
              label: 'weight',
              size: 3,
              stride: 3 * 3,
            },
            {
              label: 'channel',
              size: 3,
              stride: 3,
            },
          ],
          data_offset: 0,
        },
        data: [
          1,
          2,
          3,
          4,
          5,
          6,
          10,
          20,
          30,
          40,
          50,
          60,
          101,
          102,
          103,
          104,
          105,
          106,
        ],
      });
      console.log(`Publish ${++count} messages.`);
    }, 1000);

    rclnodejs.spin(node);
  })
  .catch(e => {
    console.log(e);
  });
