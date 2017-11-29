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
  const node = rclnodejs.createNode('colorrgba_publisher');
  const ColorRGBA = 'std_msgs/msg/ColorRGBA';
  const msg = {
    r: 127,
    g: 255,
    b: 255,
    a: 0.5,
  };

  var publisher = node.createPublisher(ColorRGBA, 'ColorRGBA_channel');
  var timer = setInterval(() => {
    publisher.publish(msg);
  }, 100);

  rclnodejs.spin(node);
  process.on('SIGINT', (m) => {
    clearInterval(timer);
    node.destroy();
    rclnodejs.shutdown();
    process.exit(0);
  });  
}).catch((err) => {
  console.log(err);
});
