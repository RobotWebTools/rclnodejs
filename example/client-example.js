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
  let node = rclnodejs.createNode('client_example_node');

  /* eslint-disable */
  let example_interfaces = rclnodejs.require('example_interfaces');
  let client = node.createClient(example_interfaces.AddTwoInts, 'add_two_ints');
  let request = new example_interfaces.AddTwoInts.Request();
  /* eslint-enable */

  request.a = 1;
  request.b = 2;

  client.sendRequest(request, (response) => {
    console.log(`Result: sum = ${response.sum}`);
  });

  rclnodejs.spin(node);
}).catch((e) => {
  console.log(`Error: ${e}`);
});
