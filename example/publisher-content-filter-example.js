// Copyright (c) 2023 Wayne Parrott. All rights reserved.
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

/* eslint-disable camelcase */

const rclnodejs = require('../index.js');

async function main() {
  await rclnodejs.init();
  const node = new rclnodejs.Node('publisher_content_filter_example_node');
  const publisher = node.createPublisher(
    'sensor_msgs/msg/Temperature',
    'temperature'
  );

  let count = 0;
  setInterval(function () {
    let temperature = (Math.random() * 100).toFixed(2);

    publisher.publish({
      header: {
        stamp: {
          sec: 123456,
          nanosec: 789,
        },
        frame_id: 'main frame',
      },
      temperature: temperature,
      variance: 0,
    });

    console.log(
      `Publish temerature message-${++count}: ${temperature} degrees`
    );
  }, 100);

  node.spin();
}

main();
