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

rclnodejs.init();

let node = rclnodejs.createNode('publisher_example_node');

let messageType = undefined;
let publisher = node.createPublisher(messageType, 'topic');

let counter = 0;
setInterval(function() {
  const message = 'hello rclnodejs ' + counter++;
  console.log('Publishing message:', message);
  publisher.publishStringMessage(message);
}, 100);

rclnodejs.spin(node);

// rclnodejs.shutdown();