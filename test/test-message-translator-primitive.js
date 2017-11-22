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

describe('Rclnodejs message translation: primitive types', function() {

  let node;
  before(function() {
    return rclnodejs.init().then(function() {
      node = rclnodejs.createNode('test_message_translation_node');
      rclnodejs.spin(node);
    });
  });

  after(function() {
    rclnodejs.shutdown();
  });

  [
    {type: 'Bool',    values: [true, false]},
    {type: 'Byte',    values: [0, 1, 2, 3, 255]},
    {type: 'Char',    values: [-128, -127, -2, -1, 0, 1, 2, 3, 127]},
    {type: 'Float32', values: [-5, 0, 1.25, 89.75, 72.50, 3.141592e8]},
    {type: 'Float64', values: [-5, 0, 1.25, 89.75, 72.50, 3.141592e8]},
    {type: 'Int16',   values: [-32768, -2, -1, 0, 1, 2, 3, 32767]},
    {type: 'Int32',   values: [-32768, -2, -1, 0, 1, 2, 3, 32767]},
    {type: 'Int64',   values: [-32768, -2, -1, 0, 1, 2, 3, 32767]},
    {type: 'Int8',    values: [-128, -127, -2, -1, 0, 1, 2, 3, 127]},
    {type: 'String',  values: ['', 'A String', ' ', '<>']},
    {type: 'UInt16',  values: [0, 1, 2, 3, 32767, 65535]},
    {type: 'UInt32',  values: [0, 1, 2, 3, 32767, 65535]},
    {type: 'UInt64',  values: [0, 1, 2, 3, 32767, 65535]},
    {type: 'UInt8',   values: [0, 1, 2, 3, 127, 255]},
  ].forEach((testData) => {
    let MessageType = rclnodejs.require('std_msgs').msg[testData.type];
    const topic = testData.topic || 'topic' + testData.type;
    testData.values.forEach((v, i) => {
      it('Test translation of ' + testData.type + ' msg, value ' + v, function() {
        const publisher = node.createPublisher(MessageType, topic);
        return new Promise((resolve, reject) => {
          const sub = node.createSubscription(MessageType, topic, (value) => {
            // For primitive types, msgs are defined as a single `.data` field
            if (value.data === v) {
              node.destroySubscription(sub);
              resolve();
            } else {
              reject('case ' + i + '. Expected: ' + v + ', Got: ' + value.data);
            }
          });
          publisher.publish(v);
          node.destroyPublisher(publisher);
        });
      });
    });
  });
});
