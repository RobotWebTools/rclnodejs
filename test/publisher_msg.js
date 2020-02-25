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

var rclType = process.argv[2];
var rclValue = eval(process.argv[3]);

rclnodejs
  .init()
  .then(() => {
    var node = rclnodejs.createNode(rclType + '_publisher');
    var msgType = rclnodejs.require('std_msgs').msg[rclType];
    var msg = new msgType();
    msg.data = rclValue;

    var publisher = node.createPublisher(msgType, rclType + '_channel');
    var timer = node.createTimer(100, () => {
      publisher.publish(msg);
    });

    rclnodejs.spin(node);
    process.on('SIGINT', m => {
      timer.cancel();
      node.destroy();
      rclnodejs.shutdown();
      process.exit(0);
    });
  })
  .catch(err => {
    console.log(err);
  });
