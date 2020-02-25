// Copyright (c) 2018 Intel Corporation. All rights reserved.
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
const deepEqual = require('deep-equal');
const rclnodejs = require('../index.js');

describe('Rclnodejs message properities validation', function() {
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
      type: 'Header',
      value: {
        stamp: { sec: 11223, nanosec: 44556 },
        frame_id: 'f001',
      },
    },
    {
      pkg: 'geometry_msgs',
      type: 'Pose',
      value: {
        position: { x: 1.5, y: 2.75, z: 3.0 },
        orientation: { x: 1.5, y: 2.75, z: 3.0, w: 1.0 },
      },
    },
    {
      pkg: 'geometry_msgs',
      type: 'Transform',
      value: {
        translation: { x: 1.5, y: 2.75, z: 3.0 },
        rotation: { x: 1.5, y: 2.75, z: 3.0, w: 1.0 },
      },
    },
    {
      pkg: 'std_msgs',
      type: 'MultiArrayDimension',
      value: {
        label: 'label name 0',
        size: 256,
        stride: 4,
      },
    },
    {
      pkg: 'sensor_msgs',
      type: 'JointState',
      value: {
        header: {
          stamp: { sec: 11223, nanosec: 44556 },
          frame_id: '1234567x',
        },
        name: ['Willy', 'Tacky'],
        position: [1, 7, 3, 4, 2, 2, 8],
        velocity: [8, 9, 6, 4],
        effort: [1, 0, 2, 6, 7],
      },
    },
    {
      pkg: 'std_msgs',
      type: 'Float32MultiArray',
      value: {
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
    },
    {
      pkg: 'std_msgs',
      type: 'Int32MultiArray',
      value: {
        layout: {
          dim: [
            { label: 'height', size: 10, stride: 600 },
            { label: 'width', size: 20, stride: 60 },
            { label: 'channel', size: 3, stride: 4 },
          ],
          data_offset: 0,
        },
        data: Int32Array.from([-10, 1, 2, 3, 8, 6, 0, -25]),
      },
    },
    {
      pkg: 'std_msgs',
      type: 'Int16MultiArray',
      value: {
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
    },
    {
      pkg: 'std_msgs',
      type: 'Int8MultiArray',
      value: {
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
    },
    {
      pkg: 'sensor_msgs',
      type: 'PointCloud',
      value: {
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
    },
  ].forEach((testData, index) => {
    it(`Test properties of ${testData.pkg}/${testData.type}.msg, case ${index}`, function() {
      const Message = rclnodejs.require(testData.pkg).msg[testData.type];
      const left = new Message();
      const right = new Message(testData.value);
      for (let i in testData.value) {
        // Assign the property with a plain object.
        left[i] = testData.value[i];
        deepEqual(left[i], right[i]);
      }
    });
  });
});
