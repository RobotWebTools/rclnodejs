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

rclnodejs
  .init()
  .then(() => {
    let node = rclnodejs.createNode('service_example_node');

    let service = node.createService(
      'example_interfaces/srv/AddTwoInts',
      'add_two_ints',
      (request, response) => {
        console.log(`Incoming request: ${typeof request}`, request);
        let result = response.template;
        result.sum = request.a + request.b;
        console.log(`Sending response: ${typeof result}`, result, '\n--');
        response.send(result);
      }
    );

    if (
      rclnodejs.DistroUtils.getDistroId() >
      rclnodejs.DistroUtils.getDistroId('humble')
    ) {
      console.log('Introspection configured');
      // To view service events use the following command:
      //    ros2 topic echo "/add_two_ints/_service_event"
      service.configureIntrospection(
        node.getClock(),
        rclnodejs.QoS.profileSystemDefault,
        rclnodejs.ServiceIntrospectionStates.CONTENTS
      );
    }

    rclnodejs.spin(node);
  })
  .catch((e) => {
    console.log(`Error: ${e}`);
  });
