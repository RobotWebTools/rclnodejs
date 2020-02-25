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
    const node = rclnodejs.createNode('client_example_node');

    const client = node.createClient(
      'example_interfaces/srv/AddTwoInts',
      'add_two_ints'
    );
    const request = {
      a: Math.floor(Math.random() * 100),
      b: Math.floor(Math.random() * 100),
    };

    client.waitForService(1000).then(result => {
      if (!result) {
        console.log('Error: service not available');
        rclnodejs.shutdown();
        return;
      }
      console.log(`Sending: ${typeof request}`, request);
      client.sendRequest(request, response => {
        console.log(`Result: ${typeof response}`, response);
        rclnodejs.shutdown();
      });
    });

    rclnodejs.spin(node);
  })
  .catch(e => {
    console.log(`Error: ${e}`);
  });
