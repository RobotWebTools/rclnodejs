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
  let MultiArrayDimension = rclnodejs.require('std_msgs').msg.MultiArrayDimension;
  let MultiArrayLayout = rclnodejs.require('std_msgs').msg.MultiArrayLayout;
  let Int32MultiArray = rclnodejs.require('std_msgs').msg.Int32MultiArray;

  const node = rclnodejs.createNode('publisher_multiarray_node');
  const publisher = node.createPublisher(Int32MultiArray, 'Int32MultiArray');
  let count = 0;

  setInterval(function() {
    // Please reference the usage of multi-array at
    // https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/MultiArrayLayout.msg
    let heightDimension = new MultiArrayDimension();
    heightDimension.label = 'height';
    heightDimension.size = 2;
    heightDimension.stride = 2 * 3 * 3;

    let weightDimension = new MultiArrayDimension();
    weightDimension.label = 'weight';
    weightDimension.size = 3;
    weightDimension.stride = 3 * 3;

    let channelDimension = new MultiArrayDimension();
    channelDimension.label = 'channel';
    channelDimension.size = 3;
    channelDimension.stride = 3;

    let layout = new MultiArrayLayout();
    layout.dim.fill([heightDimension, weightDimension, channelDimension]);
    // eslint-disable-next-line
    layout.data_offset = 0;
    let multiArray = new Int32MultiArray();
    multiArray.layout = layout;
    multiArray.data = [1, 2, 3, 4, 5, 6, 1, 2, 3, 4, 5, 6, 1, 2, 3, 4, 5, 6];

    publisher.publish(multiArray);
    console.log(`Publish ${++count} messages.`);
  }, 1000);
  rclnodejs.spin(node);
}).catch(e => {
  console.log(e);
});
