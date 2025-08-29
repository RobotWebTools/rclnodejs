// Copyright (c) 2025, The Robot Web Tools Contributors
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

const rclnodejs = require('../../../index.js');

async function main() {
  await rclnodejs.init();
  const node = rclnodejs.createNode('getmap_client_example_node');
  const client = node.createClient('nav_msgs/srv/GetMap', 'get_map');

  if (
    rclnodejs.DistroUtils.getDistroId() >
    rclnodejs.DistroUtils.getDistroId('humble')
  ) {
    // To view service events use the following command:
    //    ros2 topic echo "/get_map/_service_event"
    client.configureIntrospection(
      node.getClock(),
      rclnodejs.QoS.profileSystemDefault,
      rclnodejs.ServiceIntrospectionStates.METADATA
    );
  }

  // GetMap request is empty - no parameters needed
  const request = {};

  let result = await client.waitForService(5000);
  if (!result) {
    console.log('Error: GetMap service not available');
    console.log('Make sure to run the service first:');
    console.log('  node example/services/service/getmap-service-example.js');
    rclnodejs.shutdown();
    return;
  }

  console.log('GetMap service found. Requesting map...');
  console.log(`Sending: ${typeof request}`, request);

  client.sendRequest(request, (response) => {
    console.log(`Response received: ${typeof response}`);
    console.log('Map metadata:');
    console.log('  Frame ID:', response.map.header.frame_id);
    console.log(
      '  Timestamp:',
      response.map.header.stamp.sec + '.' + response.map.header.stamp.nanosec
    );
    console.log('  Resolution:', response.map.info.resolution, 'meters/pixel');
    console.log(
      '  Dimensions:',
      response.map.info.width + 'x' + response.map.info.height
    );
    console.log(
      '  Origin position:',
      'x:',
      response.map.info.origin.position.x,
      'y:',
      response.map.info.origin.position.y,
      'z:',
      response.map.info.origin.position.z
    );
    console.log('  Map data size:', response.map.data.length, 'cells');

    // Count different cell types
    const freeCells = response.map.data.filter((cell) => cell === 0).length;
    const occupiedCells = response.map.data.filter(
      (cell) => cell === 100
    ).length;
    const unknownCells = response.map.data.filter((cell) => cell === -1).length;

    console.log('  Cell distribution:');
    console.log('    Free cells (0):', freeCells);
    console.log('    Occupied cells (100):', occupiedCells);
    console.log('    Unknown cells (-1):', unknownCells);

    // Display a simple ASCII representation of the map (if small enough)
    if (response.map.info.width <= 20 && response.map.info.height <= 20) {
      console.log('\nASCII Map Representation:');
      console.log('  . = free space, # = occupied, ? = unknown');
      for (let y = 0; y < response.map.info.height; y++) {
        let row = '  ';
        for (let x = 0; x < response.map.info.width; x++) {
          const index = y * response.map.info.width + x;
          const cell = response.map.data[index];
          if (cell === 0) row += '.';
          else if (cell === 100) row += '#';
          else row += '?';
        }
        console.log(row);
      }
    }

    rclnodejs.shutdown();
  });

  rclnodejs.spin(node);
}

main().catch((e) => {
  console.log(`Error: ${e}`);
  rclnodejs.shutdown();
});
