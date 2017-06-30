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

// Usage: lanuch ./install/bin/examples_rclcpp_minimal_subscriber_lambda
// to display the message published by this program

'use strict';

const rclnodejs = require('../index.js');
const {message} = rclnodejs;

rclnodejs.init().then(() => {
  let node = rclnodejs.createNode('subscription_example_node');

  let messageType = message.getMessageType('std_msgs', 'msg', 'String');
  let Message = message.getMessageClass(messageType);
  messageType.Message = Message;

  node.createSubscription(messageType, 'topic', (msg) => {
    console.log(`Receive message: ${msg.data}`);
  });

  rclnodejs.spin(node);
});
