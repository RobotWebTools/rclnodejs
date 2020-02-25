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
const deepEqual = require('deep-equal');

describe('Rclnodejs message translation: complex types', function() {
  this.timeout(60 * 1000);

  before(function() {
    return rclnodejs.init();
  });

  after(function() {
    rclnodejs.shutdown();
  });

  /* eslint-disable camelcase */
  /* eslint-disable key-spacing */
  /* eslint-disable comma-spacing */

  [
    {
      pkg: 'std_msgs',
      type: 'MultiArrayDimension',
      values: [
        { label: 'label name 0', size: 256, stride: 4 },
        { label: 'label name 1', size: 48, stride: 8 },
      ],
    },
    {
      pkg: 'geometry_msgs',
      type: 'Point',
      values: [
        { x: 1.5, y: 2.75, z: 3.0 },
        { x: -1.5, y: 2.75, z: -6.0 },
      ],
    },
    {
      pkg: 'geometry_msgs',
      type: 'Point32',
      values: [
        { x: 1.5, y: 2.75, z: 3.0 },
        { x: -1.5, y: 2.75, z: -6.0 },
      ],
    },
    {
      pkg: 'geometry_msgs',
      type: 'Quaternion',
      values: [
        { x: 1.5, y: 2.75, z: 3.0, w: 1.0 },
        { x: -1.5, y: 2.75, z: -6.0, w: -1.0 },
      ],
    },
    {
      pkg: 'geometry_msgs',
      type: 'Pose',
      values: [
        {
          position: { x: 1.5, y: 2.75, z: 3.0 },
          orientation: { x: 1.5, y: 2.75, z: 3.0, w: 1.0 },
        },
        {
          position: { x: 11.5, y: 112.75, z: 9.0 },
          orientation: { x: 31.5, y: 21.5, z: 7.5, w: 1.5 },
        },
      ],
    },
    {
      pkg: 'geometry_msgs',
      type: 'Transform',
      values: [
        {
          translation: { x: 1.5, y: 2.75, z: 3.0 },
          rotation: { x: 1.5, y: 2.75, z: 3.0, w: 1.0 },
        },
        {
          translation: { x: 11.5, y: 112.75, z: 9.0 },
          rotation: { x: 31.5, y: 21.5, z: 7.5, w: 1.5 },
        },
      ],
    },
    {
      pkg: 'sensor_msgs',
      type: 'JointState',
      values: [
        {
          header: {
            stamp: { sec: 11223, nanosec: 44556 },
            frame_id: '1234567x',
          },
          name: ['Willy', 'Tacky'],
          position: [1, 7, 3, 4, 2, 2, 8],
          velocity: [8, 9, 6, 4],
          effort: [1, 0, 2, 6, 7],
        },
        {
          header: {
            stamp: { sec: 11223, nanosec: 44556 },
            frame_id: '0001234567x',
          },
          name: ['Goodly', 'Lovely', 'Angel', 'Neatly', 'Perfect', 'Tacky'],
          position: [1, 23, 7, 3, 4, 2, 2, 8],
          velocity: [1, 9, 8, 9, 6, 4],
          effort: [2, 1, 1, 0, 2, 6, 7],
        },
      ],
    },
    {
      pkg: 'std_msgs',
      type: 'Float32MultiArray',
      values: [
        {
          layout: {
            dim: [
              { label: 'height', size: 480, stride: 921600 },
              { label: 'width', size: 640, stride: 1920 },
              { label: 'channel', size: 3, stride: 8 },
            ],
            data_offset: 1024,
          },
          data: [1.0, 2.0, 3.0, 8.5, 6.75, 0.5, -0.25],
        },
        {
          layout: {
            dim: [
              { label: 'height', size: 480, stride: 921600 },
              { label: 'width', size: 640, stride: 1920 },
              { label: 'channel', size: 3, stride: 8 },
            ],
            data_offset: 1024,
          },
          data: Float32Array.from([1.0, 2.0, 3.0, 8.5, 6.75, 0.5, -0.25]),
        },
      ],
    },
    {
      pkg: 'std_msgs',
      type: 'Int32MultiArray',
      values: [
        {
          layout: {
            dim: [
              { label: 'height', size: 10, stride: 600 },
              { label: 'width', size: 20, stride: 60 },
              { label: 'channel', size: 3, stride: 4 },
            ],
            data_offset: 0,
          },
          data: [-10, 1, 2, 3, 8, 6, 0, -25], // Provide data via Array
        },
        {
          layout: {
            dim: [
              { label: 'height', size: 10, stride: 600 },
              { label: 'width', size: 20, stride: 60 },
              { label: 'channel', size: 3, stride: 4 },
            ],
            data_offset: 0,
          },
          data: Int32Array.from([-10, 1, 2, 3, 8, 6, 0, -25]), // Provide data via TypedArray
        },
      ],
    },
    {
      pkg: 'std_msgs',
      type: 'Int16MultiArray',
      values: [
        {
          layout: {
            dim: [
              { label: 'height', size: 10, stride: 600 },
              { label: 'width', size: 20, stride: 60 },
              { label: 'channel', size: 3, stride: 4 },
            ],
            data_offset: 0,
          },
          data: [-10, 1, 2, 3, 8, 6, 0, -25], // Provide data via Array
        },
        {
          layout: {
            dim: [
              { label: 'height', size: 10, stride: 600 },
              { label: 'width', size: 20, stride: 60 },
              { label: 'channel', size: 3, stride: 4 },
            ],
            data_offset: 0,
          },
          data: Int16Array.from([-10, 1, 2, 3, 8, 6, 0, -25]), // Provide data via TypedArray
        },
      ],
    },
    {
      pkg: 'std_msgs',
      type: 'Int8MultiArray',
      values: [
        {
          layout: {
            dim: [
              { label: 'height', size: 10, stride: 600 },
              { label: 'width', size: 20, stride: 60 },
              { label: 'channel', size: 3, stride: 4 },
            ],
            data_offset: 0,
          },
          data: [-10, 1, 2, 3, 8, 6, 0, -25], // Provide data via Array
        },
        {
          layout: {
            dim: [
              { label: 'height', size: 10, stride: 600 },
              { label: 'width', size: 20, stride: 60 },
              { label: 'channel', size: 3, stride: 4 },
            ],
            data_offset: 0,
          },
          data: Int8Array.from([-10, 1, 2, 3, 8, 6, 0, -25]), // Provide data via TypedArray
        },
      ],
    },
    {
      pkg: 'sensor_msgs',
      type: 'PointCloud',
      values: [
        {
          header: {
            stamp: { sec: 11223, nanosec: 44556 },
            frame_id: 'f001',
          },
          points: [
            { x: 0, y: 1, z: 3 },
            { x: 0, y: 1, z: 3 },
            { x: 0, y: 1, z: 3 },
            { x: 0, y: 1, z: 3 },
          ],
          channels: [
            {
              name: 'rgb',
              values: [0.0, 1.5, 2.0, 3.75],
            },
            {
              name: 'intensity',
              values: [10.0, 21.5, 2.0, 3.75],
            },
          ],
        },
      ],
    },
    /* eslint-enable camelcase */
    /* eslint-enable key-spacing */
    /* eslint-enable comma-spacing */
  ].forEach(testData => {
    const topic = testData.topic || 'topic' + testData.type;
    testData.values.forEach((v, i) => {
      it(
        'Test translation of ' + testData.type + ' msg, case ' + i,
        function() {
          const node = rclnodejs.createNode('test_message_translation_node');
          const MessageType = testData.pkg + '/msg/' + testData.type;
          const publisher = node.createPublisher(MessageType, topic);
          return new Promise((resolve, reject) => {
            const sub = node.createSubscription(MessageType, topic, value => {
              if (deepEqual(value, v)) {
                node.destroy();
                resolve();
              } else {
                console.log('got', value);
                console.log('expected', v);
                reject('case ' + i + '. Expected: ' + v + ', Got: ' + value);
              }
            });
            publisher.publish(v);
            rclnodejs.spin(node);
          });
        }
      );
    });
  });
});
