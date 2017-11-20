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

const assert = require('assert');
const rclnodejs = require('../index.js');

describe('Rclnodejs message translation: primitive types', function() {
  // this.timeout(60 * 1000);

  after(function() {
    rclnodejs.shutdown();
  });

  it('(Placeholder case): check the above test cases...', function(){
    return;
  });

  return rclnodejs.init().then(function(){
    let node = rclnodejs.createNode('test_message_translation_node');
    rclnodejs.spin(node);

    [
      {
        pkg: 'std_msgs',
        type: 'MultiArrayDimension',
        values: [
          {
            label:  'label name 0',
            size:   256,
            stride: 4,
          },
          {
            label:  'label name 1',
            size:   48,
            stride: 8,
          },
        ]
      },
    ].forEach((testData) => {
      let MessageType = rclnodejs.require(testData.pkg).msg[testData.type];
      const topic = testData.topic || 'topic' + testData.type;
      const publisher = node.createPublisher(MessageType, topic);
      testData.values.forEach((v, i) => {
        it('Test translation of ' + testData.type + ' msg, case ' + i, function() {
          return new Promise((resolve, reject) => {
            let sub = node.createSubscription(MessageType, topic, (value) => {
              if (rclnodejs.util.deepEqual(value, v)) {
                // Note: waiting for Issue 184 (https://github.com/RobotWebTools/rclnodejs/issues/184)
                node.destroySubscription(sub);
                resolve();
              } else {
                // Note: this routine will be invoked multiple times unless
                //       Issue 184 is resolved (https://github.com/RobotWebTools/rclnodejs/issues/184)
                reject('case ' + i + '. Expected: ' + v + ', Got: ' + value);
              }
            });
            publisher.publish(v);
          });
        });
      });
    });
  });
});
