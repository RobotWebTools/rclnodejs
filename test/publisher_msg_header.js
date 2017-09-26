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
  var node = rclnodejs.createNode('Header_publisher');
  const Header = rclnodejs.require('std_msgs').msg.Header;
  const Time = rclnodejs.require('builtin_interfaces').msg.Time;
  var time = new Time();
  time.sec = 123456;
  time.nanosec = 789;
  var header = new Header();
  header.stamp = time;
  // eslint-disable-next-line
  header.frame_id = 'main frame';

  var publisher = node.createPublisher(Header, 'Header_channel');
  var timer = setInterval(() => {
    publisher.publish(header);
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
