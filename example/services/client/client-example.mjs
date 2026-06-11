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

// From an installed package you would write `import rclnodejs from 'rclnodejs'`;
// run from this checkout we import the source entry point directly.

import rclnodejs from '../../../index.js';

await rclnodejs.init();

const node = rclnodejs.createNode('client_example_node');
const client = node.createClient(
  'example_interfaces/srv/AddTwoInts',
  'add_two_ints'
);

if (
  rclnodejs.DistroUtils.getDistroId() >
  rclnodejs.DistroUtils.getDistroId('humble')
) {
  // To view service events use the following command:
  //    ros2 topic echo "/add_two_ints/_service_event"
  client.configureIntrospection(
    node.getClock(),
    rclnodejs.QoS.profileSystemDefault,
    rclnodejs.ServiceIntrospectionStates.METADATA
  );
}

const request = {
  a: BigInt(Math.floor(Math.random() * 100)),
  b: BigInt(Math.floor(Math.random() * 100)),
};

const available = await client.waitForService(1000);
if (!available) {
  console.log('Error: service not available');
  rclnodejs.shutdown();
} else {
  console.log(`Sending: ${typeof request}`, request);
  client.sendRequest(request, (response) => {
    console.log(`Result: ${typeof response}`, response);
    rclnodejs.shutdown();
  });

  rclnodejs.spin(node);
}
