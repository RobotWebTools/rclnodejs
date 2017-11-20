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

describe('Rclnodejs message translation: complex types', function() {

  let node;
  before(function() {
    return rclnodejs.init().then(function(){
      node = rclnodejs.createNode('test_message_translation_node');
      rclnodejs.spin(node);
    });
  });

  after(function() {
    rclnodejs.shutdown();
  });

  [
    {
      pkg: 'std_msgs', type: 'MultiArrayDimension',
      values: [
        {label:  'label name 0', size:   256, stride: 4, },
        {label:  'label name 1', size:   48,  stride: 8, },
      ]
    },
    {
      pkg: 'geometry_msgs', type: 'Point',
      values: [
        {x: 1.5,  y: 2.75, z: 3.0,  },
        {x: -1.5, y: 2.75, z: -6.0, },
      ]
    },
    {
      pkg: 'geometry_msgs', type: 'Point32',
      values: [
        {x: 1.5,  y: 2.75, z: 3.0,  },
        {x: -1.5, y: 2.75, z: -6.0, },
      ]
    },
    {
      pkg: 'geometry_msgs', type: 'Quaternion',
      values: [
        {x: 1.5,  y: 2.75, z: 3.0,  w: 1.0, },
        {x: -1.5, y: 2.75, z: -6.0, w: -1.0,},
      ]
    },
    {
      pkg: 'geometry_msgs', type: 'Pose',
      values: [
        {
          position:    {x: 1.5,  y: 2.75,   z: 3.0, },
          orientation: {x: 1.5,  y: 2.75,   z: 3.0, w: 1.0, },
        },
        {
          position:    {x: 11.5, y: 112.75, z: 9.0, },
          orientation: {x: 31.5, y: 21.5,   z: 7.5, w: 1.5, },
        },
      ]
    },
    {
      pkg: 'geometry_msgs', type: 'Transform',
      values: [
        {
          translation: {x: 1.5,  y: 2.75,   z: 3.0, },
          rotation:    {x: 1.5,  y: 2.75,   z: 3.0, w: 1.0, },
        },
        {
          translation: {x: 11.5, y: 112.75, z: 9.0, },
          rotation:    {x: 31.5, y: 21.5,   z: 7.5, w: 1.5, },
        },
      ]
    },
  ].forEach((testData) => {
    let MessageType = rclnodejs.require(testData.pkg).msg[testData.type];
    const topic = testData.topic || 'topic' + testData.type;
    testData.values.forEach((v, i) => {
      it('Test translation of ' + testData.type + ' msg, case ' + i, function() {
        const publisher = node.createPublisher(MessageType, topic);
        return new Promise((resolve, reject) => {
          const sub = node.createSubscription(MessageType, topic, (value) => {
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
          node.destroyPublisher(publisher);
        });
      });
    });
  });

});
