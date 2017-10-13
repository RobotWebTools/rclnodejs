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

rclnodejs.init().then(() => {
  let Int32MultiArray = rclnodejs.require('std_msgs').msg.Int32MultiArray;
  let node = rclnodejs.createNode('subscription_multiarray_node');

  node.createSubscription(Int32MultiArray, 'Int32MultiArray', (multiArray) => {
    // Please reference the usage of multi-array at
    // https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/MultiArrayLayout.msg
    console.log('Iterate the multi array:');
    let dim = multiArray.layout.dim;
    let height = dim.data[0].size;
    let weight = dim.data[1].size;
    let weightStride = dim.data[1].stride;
    let channel = dim.data[2].size;
    let channelStride = dim.data[2].stride;
    // eslint-disable-next-line
    let offset = multiArray.layout.data_offset

    for (let i = 0; i < height; i++) {
      for (let j = 0; j < weight; j++) {
        for (let k = 0; k < channel; k++) {
          console.log(`multiarray(${i},${j},${k}) = ${multiArray.data[offset + weightStride*i + channelStride*j + k]}`);
        }
      }
    }
  });
  rclnodejs.spin(node);
}).catch(e => {
  console.log(e);
});
