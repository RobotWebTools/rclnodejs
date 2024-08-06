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

describe('Test message which has a fixed array of 36', function () {
  this.timeout(60 * 1000);

  const mapData = {
    map: {
      header: {
        stamp: {
          sec: 123456,
          nanosec: 789,
        },
        frame_id: 'main_frame',
      },
      info: {
        map_load_time: {
          sec: 123456,
          nanosec: 789,
        },
        resolution: 1.0,
        width: 1024,
        height: 768,
        origin: {
          position: {
            x: 0.0,
            y: 0.0,
            z: 0.0,
          },
          orientation: {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 0.0,
          },
        },
      },
      data: Int8Array.from([1, 2, 3]),
    },
    initial_pose: {
      header: {
        stamp: {
          sec: 123456,
          nanosec: 789,
        },
        frame_id: 'main frame',
      },
      pose: {
        pose: {
          position: { x: 11.5, y: 112.75, z: 9.0 },
          orientation: { x: 31.5, y: 21.5, z: 7.5, w: 1.5 },
        },
        covariance: Float64Array.from({ length: 36 }, (v, k) => k),
      },
    },
  };

  before(function () {
    return rclnodejs.init();
  });

  after(function () {
    rclnodejs.shutdown();
  });

  it('Assigned with an array of 36', function (done) {
    const node = rclnodejs.createNode('set_map_client');
    node.createService(
      'nav_msgs/srv/SetMap',
      'set_map',
      (request, response) => {
        assert.deepStrictEqual(request, mapData);
        response.success = true;
        return response;
      }
    );

    rclnodejs.spin(node);
    const client = node.createClient('nav_msgs/srv/SetMap', 'set_map');
    client.sendRequest(mapData, (response) => {
      assert.deepStrictEqual(response.success, true);
      node.destroy();
      done();
    });
  });

  it('Assigned with a longer array', function (done) {
    mapData.initial_pose.pose.covariance = Float64Array.from(
      { length: 37 },
      (v, k) => k
    );
    const node = rclnodejs.createNode('set_map_client');
    const client = node.createClient('nav_msgs/srv/SetMap', 'set_map');
    assert.throws(() => {
      client.sendRequest(mapData, (response) => { });
    }, RangeError);
    node.destroy();
    done();
  });

  it('Assigned with a shorter array', function (done) {
    mapData.initial_pose.pose.covariance = Float64Array.from(
      { length: 35 },
      (v, k) => k
    );
    const node = rclnodejs.createNode('set_map_client');
    const client = node.createClient('nav_msgs/srv/SetMap', 'set_map');
    assert.throws(() => {
      client.sendRequest(mapData, (response) => { });
    }, RangeError);
    node.destroy();
    done();
  });

  it('Tested with different kinds of fixed array', function (done) {
    const bacicType = {
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

    const defaultValue = {
      bool_value: true,
      byte_value: 50,
      char_value: 100,
      float32_value: 1.125,
      float64_value: 1.125,
      int8_value: -50,
      uint8_value: 200,
      int16_value: -1000,
      uint16_value: 2000,
      int32_value: -30000,
      uint32_value: 60000,
      int64_value: -40000000,
      uint64_value: 50000000,
    };

    const msg = {
      bool_values: [true, false, true],
      byte_values: Uint8Array.from([127, 125, 100]),
      char_values: Int8Array.from([127, 125, 100]),
      float32_values: Float32Array.from([1.1, 2.2, 3.3]),
      float64_values: Float64Array.from([1.1, 2.2, 3.3]),
      int8_values: Int8Array.from([1, 2, 3]),
      uint8_values: Uint8Array.from([1, 2, 3]),
      int16_values: Int16Array.from([1, 2, 3]),
      uint16_values: Uint16Array.from([1, 2, 3]),
      int32_values: Int32Array.from([1, 2, 3]),
      uint32_values: Uint32Array.from([1, 2, 3]),
      int64_values: [1, 2, 3],
      uint64_values: [1, 2, 3],
      string_values: ['hello', 'world', 'abc'],
      basic_types_values: [bacicType, bacicType, bacicType],
      defaults_values: [defaultValue, defaultValue, defaultValue],
      // Use a string, '18446744073709551615', representing UINT64_MAX for ref,
      // see details https://github.com/node-ffi-napi/ref-napi/blob/latest/test/uint64.js#L17.
      uint64_values_default: [0, 1, '18446744073709551615'],
      alignment_check: 100,
    };

    // Arrays.msg defined https://github.com/ros2/test_interface_files/blob/rolling/msg/Arrays.msg
    const expected = {
      bool_values: [true, false, true],
      byte_values: Uint8Array.from([127, 125, 100]),
      char_values: Int8Array.from([127, 125, 100]),
      float32_values: Float32Array.from([1.1, 2.2, 3.3]),
      float64_values: Float64Array.from([1.1, 2.2, 3.3]),
      int8_values: Int8Array.from([1, 2, 3]),
      uint8_values: Uint8Array.from([1, 2, 3]),
      int16_values: Int16Array.from([1, 2, 3]),
      uint16_values: Uint16Array.from([1, 2, 3]),
      int32_values: Int32Array.from([1, 2, 3]),
      uint32_values: Uint32Array.from([1, 2, 3]),
      int64_values: [1, 2, 3],
      uint64_values: [1, 2, 3],
      string_values: ['hello', 'world', 'abc'],
      basic_types_values: [bacicType, bacicType, bacicType],
      defaults_values: [defaultValue, defaultValue, defaultValue],
      bool_values_default: [false, true, false],
      byte_values_default: Uint8Array.from([0, 1, 255]),
      char_values_default: Int8Array.from([0, 1, 127]),
      float32_values_default: Float32Array.from([1.125, 0.0, -1.125]),
      float64_values_default: Float64Array.from([3.1415, 0.0, -3.1415]),
      int8_values_default: Int8Array.from([0, 127, -128]),
      uint8_values_default: Uint8Array.from([0, 1, 255]),
      int16_values_default: Int16Array.from([0, 32767, -32768]),
      uint16_values_default: Uint16Array.from([0, 1, 65535]),
      int32_values_default: Int32Array.from([0, 2147483647, -2147483648]),
      uint32_values_default: Uint32Array.from([0, 1, 4294967295]),
      int64_values_default: [0, 9223372036854775807, -9223372036854775808],
      uint64_values_default: [0, 1, '18446744073709551615'],
      string_values_default: ["", "max value", "min value"],
      alignment_check: 100,
      constants_values: [],
    };

    const node = rclnodejs.createNode('fixed_arrays');
    let publisher = node.createPublisher(
      'test_msgs/msg/Arrays',
      'fixed_arrays'
    );
    let timer = setInterval(() => {
      assert.doesNotThrow(() => {
        publisher.publish(msg);
      }, RangeError);
    }, 100);

    node.createSubscription(
      'test_msgs/msg/Arrays',
      'fixed_arrays',
      (response) => {
        clearInterval(timer);
        assert.deepEqual(response, expected);
        node.destroy();
        done();
      }
    );

    rclnodejs.spin(node);
  });
});
