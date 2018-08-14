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
const rclnodejs = require('../index.js');

/* eslint-disable camelcase */
/* eslint-disable key-spacing */
/* eslint-disable comma-spacing */
describe('Test bounded array primitive', function() {
  this.timeout(60 * 1000);
  const msg = {
    bool_values: [true, false],
    byte_values: Uint8Array.from([127, 125]),
    char_values: Int8Array.from([127, 125]),
    float32_values: Float32Array.from([1.1, 2.2, 3.3]),
    float64_values: Float64Array.from([1.1, 2.2]),
    int8_values: Int8Array.from([1, 2]),
    uint8_values: Uint8Array.from([1, 2]),
    int16_values: Int16Array.from([1, 2]),
    uint16_values: Uint16Array.from([1, 2]),
    int32_values: Int32Array.from([1, 2]),
    uint32_values: Uint32Array.from([1, 2]),
    int64_values: [1, 2],
    uint64_values: [1, 2],
    string_values: ['hello', 'world'],
    check: 100
  };

  before(function() {
    return rclnodejs.init();
  });

  after(function() {
    rclnodejs.shutdown();
  });

  it('Assigned with an array with length <= 3', function(done) {
    const node = rclnodejs.createNode('bounded_array');
    let publisher = node.createPublisher('test_msgs/msg/BoundedArrayPrimitives', 'bounded_array');

    let timer = setInterval(() => {
      assert.doesNotThrow(() => {
        publisher.publish(msg);
      }, RangeError);
    }, 100);

    node.createSubscription('test_msgs/msg/BoundedArrayPrimitives', 'bounded_array', (response) => {
      clearInterval(timer);
      assert.deepStrictEqual(response, msg);
      node.destroy();
      done();
    });

    rclnodejs.spin(node);
  });

  it('Assigned with an array with length > 3', function(done) {
    let msgCopy = Object.create(msg);

    // Assigned with array whose length excessed the limitation
    msgCopy.float32_values = Float32Array.from([1.1, 2.2, 3.3, 6.6]);
    const node = rclnodejs.createNode('bounded_array');
    let publisher = node.createPublisher('test_msgs/msg/BoundedArrayPrimitives', 'bounded_array');
    assert.throws(() => {
      publisher.publish(msgCopy);
    }, RangeError);
    node.destroy();
    done();
  });
});

describe('Test bounded array nested', function() {
  this.timeout(60 * 1000);
  const primitives = {
    bool_value: true,
    byte_value: 127,
    char_value: 97,
    float32_value: 1.25,
    float64_value: 2.2,
    int8_value: 1,
    uint8_value: 2,
    int16_value: 2,
    uint16_value: 1,
    int32_value: 1,
    uint32_value: 1,
    int64_value: 2,
    uint64_value: 2,
    string_value: 'hello'
  };

  before(function() {
    return rclnodejs.init();
  });

  after(function() {
    rclnodejs.shutdown();
  });

  it('Assigned an array with length <= 4', function(done) {
    const msg = {primitive_values: [primitives, primitives, primitives]};
    const node = rclnodejs.createNode('bounded_array');
    let timer = setInterval(() => {
      let publisher = node.createPublisher('test_msgs/msg/BoundedArrayNested', 'bounded_array');
      assert.doesNotThrow(() => {
        publisher.publish(msg);
      }, RangeError);
    }, 100);

    node.createSubscription('test_msgs/msg/BoundedArrayNested', 'bounded_array', (response) => {
      clearInterval(timer);
      assert.deepStrictEqual(response, msg);
      node.destroy();
      done();
    });

    rclnodejs.spin(node);
  });

  it('Assigned an array with length > 4', function(done) {
    // Assigned with array whose length excessed the limitation
    let msg = {primitive_values: [primitives, primitives, primitives, primitives, primitives]};
    const node = rclnodejs.createNode('bounded_array');
    let publisher = node.createPublisher('test_msgs/msg/BoundedArrayNested', 'bounded_array');
    assert.throws(() => {
      publisher.publish(msg);
    }, RangeError);
    node.destroy();
    done();
  });
});
