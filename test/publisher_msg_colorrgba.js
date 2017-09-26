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
  var node = rclnodejs.createNode('colorrgba_publisher');
  var ColorRGBA = rclnodejs.require('std_msgs').msg.ColorRGBA;
  var msg = new ColorRGBA();
  msg.r = 127;
  msg.g = 255;
  msg.b = 255;
  msg.a = 0.5;

  var publisher = node.createPublisher(ColorRGBA, 'ColorRGBA_channel');
  var timer = setInterval(() => {
    publisher.publish(msg);
  }, 100);

  rclnodejs.spin(node);
  process.on('message', (m) => {
    clearInterval(timer);
    node.destroy();
    rclnodejs.shutdown();
    process.exit(0);
  });  
}).catch((err) => {
  console.log(err);
});
