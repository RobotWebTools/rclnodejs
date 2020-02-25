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
const { validator } = rclnodejs;

console.log(
  'validation of topic "/node_name/chatter" is ' +
    validator.validateFullTopicName('/node_name/chatter')
);
console.log(
  'validation of node "my_node" is ' + validator.validateNodeName('my_node')
);
console.log(
  'validation of topic "chatter" is ' + validator.validateTopicName('chatter')
);
console.log(
  'validation of namespace "/my_ns" is ' + validator.validateNamespace('/my_ns')
);
