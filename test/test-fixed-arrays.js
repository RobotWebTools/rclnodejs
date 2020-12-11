/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

'use strict';

const assert = require('assert');
const rclnodejs = require('../index.js');

describe('rclnodejs message communication', function () {
  this.timeout(60 * 1000);

  before(function () {
    return rclnodejs.init();
  });

  after(function () {
    rclnodejs.shutdown();
  });

  it('should publish messages with fixed arrays', function (done) {
    const publisherNode = rclnodejs.createNode('fixed_arrays_publisher');
    const subscriptionNode = rclnodejs.createNode('fixed_arrays_subscriber');
    const originalMsg = {
      bool_values: [true, true, false],
      byte_values: [0, 1, 2],
      char_values: [0, 1, 2],
      float32_values: [0, 1, 2],
      float64_values: [0, 1, 2],
      int8_values: [0, 1, 2],
      uint8_values: [0, 1, 2],
      int16_values: [0, 1, 2],
      uint16_values: [0, 1, 2],
      int32_values: [0, 1, 2],
      uint32_values: [0, 1, 2],
      int64_values: [0, 1, 2],
      uint64_values: [0, 1, 2],
      string_values: ['a', 'b', 'c'],
      basic_types_values: [{}, {}, {}],
      constants_values: [{}, {}, {}],
      defaults_values: [{}, {}, {}],
      alignment_check: 0,
    };

    const subscription = subscriptionNode.createSubscription(
      'test_msgs/msg/Arrays',
      'fixed_arrays_channel1',
      (msg) => {
        timer.cancel();
        assert.deepStrictEqual(msg.bool_values, originalMsg.bool_values);
        assert.deepStrictEqual(msg.byte_values, originalMsg.byte_values);
        assert.deepStrictEqual(msg.char_values, originalMsg.char_values);
        assert.deepStrictEqual(msg.float32_values, originalMsg.float32_values);
        assert.deepStrictEqual(msg.float64_values, originalMsg.float64_values);
        assert.deepStrictEqual(msg.int8_values, originalMsg.int8_values);
        assert.deepStrictEqual(msg.uint8_values, originalMsg.uint8_values);
        assert.deepStrictEqual(msg.int16_values, originalMsg.int16_values);
        assert.deepStrictEqual(msg.uint16_values, originalMsg.uint16_values);
        assert.deepStrictEqual(msg.int32_values, originalMsg.int32_values);
        assert.deepStrictEqual(msg.uint32_values, originalMsg.uint32_values);
        assert.deepStrictEqual(msg.int64_values, originalMsg.int64_values);
        assert.deepStrictEqual(msg.uint64_values, originalMsg.uint64_values);
        assert.deepStrictEqual(msg.string_values, originalMsg.string_values);
        assert.strictEqual(msg.alignment_check, 0);
        assert.strictEqual(msg.defaults_values[0].bool_value, true);
        assert.strictEqual(msg.defaults_values[1].bool_value, true);
        assert.strictEqual(msg.defaults_values[2].bool_value, true);
        assert.strictEqual(msg.defaults_values[0].int8_value, -50);
        assert.strictEqual(msg.defaults_values[1].int8_value, -50);
        assert.strictEqual(msg.defaults_values[2].int8_value, -50);
        publisherNode.destroy();
        subscriptionNode.destroy();
        done();
      }
    );

    const publisher = publisherNode.createPublisher(
      'test_msgs/msg/Arrays',
      'fixed_arrays_channel1'
    );
    const timer = publisherNode.createTimer(100, () => {
      publisher.publish(originalMsg);
    });
    rclnodejs.spin(subscriptionNode);
    rclnodejs.spin(publisherNode);
  });
});
