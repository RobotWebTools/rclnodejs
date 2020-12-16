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
describe('Test bounded sequeces', function () {
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
  };

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
    int64_values_default: [-1, 2, 3],
    uint64_values_default: [1, 2, 3],
    string_values: ['hello', 'world'],
    basic_types_values: [primitives, primitives],
    alignment_check: 100,
  };

  const expected = {
    bool_values: [true, false],
    byte_values: Uint8Array.from([127, 125]),
    char_values: Int8Array.from([127, 125]), // char[] is uint8_t[] in ROS, the old serialization mistakenly deserialized it as Int8Array
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
    int64_values_default: [-1, 2, 3],
    uint64_values_default: [1, 2, 3],
    string_values: ['hello', 'world'],
    basic_types_values: [primitives, primitives],
    bool_values_default: [false, true, false],
    byte_values_default: Uint8Array.from([0, 1, 255]),
    char_values_default: Int8Array.from([0, 1, 127]),
    float32_values_default: Float32Array.from([1.125, 0, -1.125]),
    float64_values_default: Float64Array.from([3.1415, 0, -3.1415]),
    int16_values_default: Int16Array.from([0, 32767, -32768]),
    int32_values_default: Int32Array.from([0, 2147483647, -2147483648]),
    int8_values_default: Int8Array.from([0, 127, -128]),
    string_values_default: ['', 'max value', 'min value'],
    uint16_values_default: Uint16Array.from([0, 1, 65535]),
    uint32_values_default: Uint32Array.from([0, 1, 4294967295]),
    uint8_values_default: Uint8Array.from([0, 1, 255]),
    alignment_check: 100,
    constants_values: [],
    defaults_values: [],
  };

  before(function () {
    return rclnodejs.init();
  });

  after(function () {
    rclnodejs.shutdown();
  });

  it('Assigned with bounded sequences', function (done) {
    const node = rclnodejs.createNode('bounded_sequences');
    let publisher = node.createPublisher(
      'test_msgs/msg/BoundedSequences',
      'bounded_sequences'
    );
    let timer = setInterval(() => {
      assert.doesNotThrow(() => {
        publisher.publish(msg);
      }, RangeError);
    }, 100);

    node.createSubscription(
      'test_msgs/msg/BoundedSequences',
      'bounded_sequences',
      (response) => {
        clearInterval(timer);
        assert.deepStrictEqual(response.bool_values, expected.bool_values);
        assert.deepStrictEqual(response.byte_values, expected.byte_values);
        assert.deepStrictEqual(
          Array.from(response.char_values),
          Array.from(expected.char_values)
        );
        assert.deepStrictEqual(
          response.float32_values,
          expected.float32_values
        );
        assert.deepStrictEqual(
          response.float64_values,
          expected.float64_values
        );
        assert.deepStrictEqual(response.int8_values, expected.int8_values);
        assert.deepStrictEqual(response.uint8_values, expected.uint8_values);
        assert.deepStrictEqual(response.int16_values, expected.int16_values);
        assert.deepStrictEqual(response.uint16_values, expected.uint16_values);
        assert.deepStrictEqual(response.int32_values, expected.int32_values);
        assert.deepStrictEqual(response.uint32_values, expected.uint32_values);
        assert.deepStrictEqual(
          Array.from(response.int64_values).map((i) => Number(i)),
          expected.int64_values
        );
        assert.deepStrictEqual(
          Array.from(response.uint64_values).map((i) => Number(i)),
          expected.uint64_values
        );
        assert.deepStrictEqual(
          Array.from(response.int64_values_default).map((i) => Number(i)),
          expected.int64_values_default
        );
        assert.deepStrictEqual(
          Array.from(response.uint64_values_default).map((i) => Number(i)),
          expected.uint64_values_default
        );
        assert.deepStrictEqual(response.string_values, expected.string_values);
        assert.deepStrictEqual(
          response.basic_types_values,
          expected.basic_types_values
        );
        assert.deepStrictEqual(
          response.bool_values_default,
          expected.bool_values_default
        );
        assert.deepStrictEqual(
          response.byte_values_default,
          expected.byte_values_default
        );
        assert.deepStrictEqual(
          Array.from(response.char_values_default),
          Array.from(expected.char_values_default)
        );
        assert.deepStrictEqual(
          response.float32_values_default,
          expected.float32_values_default
        );
        assert.deepStrictEqual(
          response.float64_values_default,
          expected.float64_values_default
        );
        assert.deepStrictEqual(
          response.int16_values_default,
          expected.int16_values_default
        );
        assert.deepStrictEqual(
          response.int32_values_default,
          expected.int32_values_default
        );
        assert.deepStrictEqual(
          response.int8_values_default,
          expected.int8_values_default
        );
        assert.deepStrictEqual(
          response.string_values_default,
          expected.string_values_default
        );
        assert.deepStrictEqual(
          response.uint16_values_default,
          expected.uint16_values_default
        );
        assert.deepStrictEqual(
          response.uint32_values_default,
          expected.uint32_values_default
        );
        assert.deepStrictEqual(
          response.uint8_values_default,
          expected.uint8_values_default
        );
        assert.deepStrictEqual(
          response.alignment_check,
          expected.alignment_check
        );
        assert.deepStrictEqual(
          response.constants_values,
          expected.constants_values
        );
        assert.deepStrictEqual(
          response.defaults_values,
          expected.defaults_values
        );
        node.destroy();
        done();
      }
    );

    rclnodejs.spin(node);
  });
});
