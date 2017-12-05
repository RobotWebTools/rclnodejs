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

/* eslint-disable camelcase */
describe('Rclnodejs non primitive message type testing', function() {
  this.timeout(60 * 1000);

  before(function() {
    return rclnodejs.init();
  });

  after(function() {
    rclnodejs.shutdown();
  });

  it('geometry_msgs/msg/Point checking', function() {
    const Point = rclnodejs.require('geometry_msgs/msg/Point');

    let point = new Point();
    point.x = 1.5;
    point.y = 2.75;
    point.z = -0.5;

    let pointClone = new Point(point);
    assert.deepStrictEqual(1.5, pointClone.x);
    assert.deepStrictEqual(2.75, pointClone.y);
    assert.deepStrictEqual(-0.5, pointClone.z);
  });

  it('sensor_msgs/msg/JointState checking', function() {
    const JointState = rclnodejs.require('sensor_msgs/msg/JointState');

    let jointState = new JointState();
    jointState.header.stamp.sec = 11223;
    jointState.header.stamp.nanosec = 44556;
    jointState.header.frame_id = '1234567x';
    jointState.name = ['Willy', 'Tacky'];
    jointState.position = [1, 7, 3, 4, 2, 2, 8];
    jointState.velocity = [8, 9, 6, 4];
    jointState.effort = [1, 0, 2, 6, 7];

    let jointStateClone = new JointState(jointState);
    assert.deepStrictEqual(11223, jointStateClone.header.stamp.sec);
    assert.deepStrictEqual(44556, jointStateClone.header.stamp.nanosec);
    assert.deepStrictEqual('1234567x', jointStateClone.header.frame_id);
    assert.deepStrictEqual(['Willy', 'Tacky'], jointStateClone.name);
    assert.deepStrictEqual(Float64Array.from([1, 7, 3, 4, 2, 2, 8]), jointStateClone.position);
    assert.deepStrictEqual(Float64Array.from([8, 9, 6, 4]), jointStateClone.velocity);
    assert.deepStrictEqual(Float64Array.from([1, 0, 2, 6, 7]), jointStateClone.effort);
  });

  it('geometry_msgs/msg/Transform checking', function() {
    const Transform = rclnodejs.require('geometry_msgs/msg/Transform');

    let transform = new Transform();
    transform.translation.x = 1.5;
    transform.translation.y = 2.75;
    transform.translation.z = 3.0;
    transform.rotation.x = 1.5;
    transform.rotation.y = 2.75;
    transform.rotation.z = 3.0;
    transform.rotation.w = 1.0;

    let transformClone = new Transform(transform);
    assert.deepStrictEqual(1.5, transformClone.translation.x);
    assert.deepStrictEqual(2.75, transformClone.translation.y);
    assert.deepStrictEqual(3.0, transformClone.translation.z);
    assert.deepStrictEqual(1.5, transformClone.rotation.x);
    assert.deepStrictEqual(2.75, transformClone.rotation.y);
    assert.deepStrictEqual(3.0, transformClone.rotation.z);
    assert.deepStrictEqual(1.0, transformClone.rotation.w);
  });

  it('std_msgs/msg/Float32MultiArray checking', function() {
    const Float32MultiArray = rclnodejs.require('std_msgs/msg/Float32MultiArray');
    const MultiArrayDimension = rclnodejs.require('std_msgs/msg/MultiArrayDimension');

    let float32MultiArray = new Float32MultiArray();
    let heightDimension = new MultiArrayDimension();
    heightDimension.label = 'height';
    heightDimension.size = 480;
    heightDimension.stride = 921600;

    let weightDimension = new MultiArrayDimension();
    weightDimension.label = 'weight';
    weightDimension.size = 640;
    weightDimension.stride = 1920;

    let channelDimension = new MultiArrayDimension();
    channelDimension.label = 'channel';
    channelDimension.size = 3;
    channelDimension.stride = 8;

    float32MultiArray.layout.dim.fill([heightDimension, weightDimension, channelDimension]);
    float32MultiArray.layout.data_offset = 1024;
    float32MultiArray.data = [1.0, 2.0, 3.0, 8.5, 6.75, 0.5, -0.25];

    let float32MultiArrayClone = new Float32MultiArray(float32MultiArray);
    assert.deepStrictEqual('height', float32MultiArrayClone.layout.dim.data[0].label);
    assert.deepStrictEqual(480, float32MultiArrayClone.layout.dim.data[0].size);
    assert.deepStrictEqual(921600, float32MultiArrayClone.layout.dim.data[0].stride);
    assert.deepStrictEqual('weight', float32MultiArrayClone.layout.dim.data[1].label);
    assert.deepStrictEqual(640, float32MultiArrayClone.layout.dim.data[1].size);
    assert.deepStrictEqual(1920, float32MultiArrayClone.layout.dim.data[1].stride);
    assert.deepStrictEqual('channel', float32MultiArrayClone.layout.dim.data[2].label);
    assert.deepStrictEqual(3, float32MultiArrayClone.layout.dim.data[2].size);
    assert.deepStrictEqual(8, float32MultiArrayClone.layout.dim.data[2].stride);
    assert.deepStrictEqual(1024, float32MultiArrayClone.layout.data_offset);
    assert.deepStrictEqual(Float32Array.from([1.0, 2.0, 3.0, 8.5, 6.75, 0.5, -0.25]), float32MultiArrayClone.data);
  });
});
