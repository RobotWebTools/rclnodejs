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

const rclnodejs = require('../index.js');

async function main() {
  await rclnodejs.init();
  const node = new rclnodejs.Node('subscription_service_event_example_node');
  let count = 0;

  node.createSubscription(
    'example_interfaces/srv/AddTwoInts_Event',
    '/add_two_ints/_service_event',
    (event) => {
      console.log(`Received event No. ${++count}: `, event);
    }
  );

  node.spin();
}

main();
