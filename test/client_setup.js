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
  .then(function() {
    var node = rclnodejs.createNode('client');
    const AddTwoInts = 'example_interfaces/srv/AddTwoInts';
    const Int8 = 'std_msgs/msg/Int8';
    var client = node.createClient(AddTwoInts, 'add_two_ints');
    const request = {
      a: 1,
      b: 2,
    };
    var publisher = node.createPublisher(Int8, 'back_add_two_ints');
    client.waitForService().then(() => {
      client.sendRequest(request, response => {
        publisher.publish(response.sum);
      });
    });

    rclnodejs.spin(node);

    process.on('SIGINT', function() {
      timer.cancel();
      node.destroy();
      rclnodejs.shutdown();
      process.exit(0);
    });
  })
  .catch(function(err) {
    console.log(err);
  });
