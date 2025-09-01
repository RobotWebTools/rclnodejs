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

rclnodejs
  .init()
  .then(() => {
    let node = rclnodejs.createNode('getmap_service_example_node');

    // Create sample map data
    const mapWidth = 10;
    const mapHeight = 10;
    const mapData = Int8Array.from({ length: mapWidth * mapHeight }, (v) => 0); // 0 = free space

    // Add some obstacles (100 = occupied, -1 = unknown)
    mapData[22] = 100; // obstacle at position (2,2)
    mapData[33] = 100; // obstacle at position (3,3)
    mapData[44] = 100; // obstacle at position (4,4)

    const sampleMapResponse = {
      map: {
        header: {
          stamp: {
            sec: Math.floor(Date.now() / 1000),
            nanosec: (Date.now() % 1000) * 1000000,
          },
          frame_id: 'map',
        },
        info: {
          map_load_time: {
            sec: Math.floor(Date.now() / 1000),
            nanosec: (Date.now() % 1000) * 1000000,
          },
          resolution: 0.05, // meters per pixel
          width: mapWidth,
          height: mapHeight,
          origin: {
            position: {
              x: -2.5,
              y: -2.5,
              z: 0.0,
            },
            orientation: {
              x: 0.0,
              y: 0.0,
              z: 0.0,
              w: 1.0,
            },
          },
        },
        data: mapData,
      },
    };

    let service = node.createService(
      'nav_msgs/srv/GetMap',
      'get_map',
      (request, response) => {
        console.log(`Incoming GetMap request: ${typeof request}`, request);
        console.log('Sending map with dimensions:', mapWidth, 'x', mapHeight);
        console.log(
          'Map resolution:',
          sampleMapResponse.map.info.resolution,
          'meters/pixel'
        );
        console.log(
          'Number of occupied cells:',
          mapData.filter((cell) => cell === 100).length
        );

        // Update timestamp to current time
        const now = Date.now();
        sampleMapResponse.map.header.stamp.sec = Math.floor(now / 1000);
        sampleMapResponse.map.header.stamp.nanosec = (now % 1000) * 1000000;

        let result = response.template;
        result.map = sampleMapResponse.map;
        console.log(
          `Sending response: ${typeof result}`,
          'Map data size:',
          result.map.data.length,
          '\n--'
        );
        response.send(result);
      }
    );

    if (
      rclnodejs.DistroUtils.getDistroId() >
      rclnodejs.DistroUtils.getDistroId('humble')
    ) {
      console.log('Introspection configured');
      // To view service events use the following command:
      //    ros2 topic echo "/get_map/_service_event"
      service.configureIntrospection(
        node.getClock(),
        rclnodejs.QoS.profileSystemDefault,
        rclnodejs.ServiceIntrospectionStates.CONTENTS
      );
    }

    console.log('GetMap service is ready. Waiting for requests...');
    console.log(
      'Service provides a',
      mapWidth + 'x' + mapHeight,
      'occupancy grid map'
    );
    rclnodejs.spin(node);
  })
  .catch((e) => {
    console.log(`Error: ${e}`);
  });
