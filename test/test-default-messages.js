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

  it('should publish messages with unfilled defaults', function (done) {
    const publisherNode = rclnodejs.createNode('defaults_message_publisher');
    const subscriptionNode = rclnodejs.createNode(
      'defaults_message_subscriber'
    );

    const subscription = subscriptionNode.createSubscription(
      'test_msgs/msg/Defaults',
      'defaults_message_channel1',
      (msg) => {
        timer.cancel();
        assert.strictEqual(msg.bool_value, true);
        assert.strictEqual(msg.byte_value, 50);
        assert.strictEqual(msg.char_value, 100);
        assert.strictEqual(msg.float32_value, 1.125);
        assert.strictEqual(msg.float64_value, 1.125);
        assert.strictEqual(msg.int8_value, -50);
        assert.strictEqual(msg.uint8_value, 200);
        assert.strictEqual(msg.int16_value, -1000);
        assert.strictEqual(msg.uint16_value, 2000);
        assert.strictEqual(msg.int32_value, -30000);
        assert.strictEqual(msg.uint32_value, 60000);
        assert.strictEqual(msg.int64_value, -40000000);
        assert.strictEqual(msg.uint64_value, 50000000);
        publisherNode.destroy();
        subscriptionNode.destroy();
        done();
      }
    );

    var publisher = publisherNode.createPublisher(
      'test_msgs/msg/Defaults',
      'defaults_message_channel1'
    );
    var timer = publisherNode.createTimer(100, () => {
      publisher.publish({});
    });
    rclnodejs.spin(subscriptionNode);
    rclnodejs.spin(publisherNode);
  });
});
