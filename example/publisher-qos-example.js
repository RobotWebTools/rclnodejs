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
const { QoS } = rclnodejs;

rclnodejs.init().then(() => {
  const node = rclnodejs.createNode('publisher_qos_example_node');

  let qos = new QoS();
  qos.hitory = QoS.HistoryPolicy.RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT;
  qos.reliability =
    QoS.ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
  qos.durability =
    QoS.DurabilityPolicy.RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;
  qos.depth = 1;
  qos.avoidRosNameSpaceConventions = false;

  const publisher = node.createPublisher('std_msgs/msg/String', 'topic', {
    qos,
  });

  let counter = 0;
  setInterval(function() {
    console.log(`Publishing message: Hello ROS ${counter}`);
    publisher.publish(`Hello ROS ${counter++}`);
  }, 1000);

  rclnodejs.spin(node);
});
