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

import assert from 'assert';
import rclnodejs from '../index.js';

describe('rclnodejs publisher test suite', function () {
  this.timeout(60 * 1000);

  beforeEach(function () {
    return rclnodejs.init();
  });

  afterEach(function () {
    rclnodejs.shutdown();
  });

  it('Try creating a publisher', function () {
    const node = rclnodejs.createNode('publisher_node');
    const String = 'std_msgs/msg/String';
    const publisher = node.createPublisher(String, 'topic');
    rclnodejs.spin(node);
  });

  it('Try publish a message', function () {
    const node = rclnodejs.createNode('publisher_node');
    const String = 'std_msgs/msg/String';
    const publisher = node.createPublisher(String, 'topic');
    const msg = 'Hello ROS 2.0 Publisher!';
    publisher.publish(msg);
    rclnodejs.spin(node);
  });

  it('Test count of subscriptions', function () {
    const node = rclnodejs.createNode('publisher_node');
    const String = 'std_msgs/msg/String';
    const publisher = node.createPublisher(String, 'topic');
    node.createSubscription(String, 'topic', (msg) => {});
    assert.strictEqual(publisher.subscriptionCount, 1);
  });

  it('Test loggerName', function () {
    const node = rclnodejs.createNode('publisher_node');
    const String = 'std_msgs/msg/String';
    const publisher = node.createPublisher(String, 'topic');
    assert.strictEqual(typeof publisher.loggerName, 'string');
  });

  it('Wait for all acked', function () {
    const node = rclnodejs.createNode('publisher_node');
    const String = 'std_msgs/msg/String';
    const publisher = node.createPublisher(String, 'topic');
    node.createSubscription(String, 'topic', (msg) => {});
    assert.strictEqual(publisher.subscriptionCount, 1);

    publisher.publish('Hello World');
    assert.strictEqual(publisher.waitForAllAcked(BigInt(1000000000)), true);
  });

  it('Test assertLiveliness', function () {
    const node = rclnodejs.createNode('publisher_node');
    const String = 'std_msgs/msg/String';
    const qos = new rclnodejs.QoS(
      rclnodejs.QoS.HistoryPolicy.RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT,
      0,
      rclnodejs.QoS.ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
      rclnodejs.QoS.DurabilityPolicy.RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT,
      rclnodejs.QoS.LivelinessPolicy.RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC
    );
    const publisher = node.createPublisher(String, 'topic', { qos: qos });
    assert.doesNotThrow(() => {
      publisher.assertLiveliness();
    });
    rclnodejs.spin(node);
  });
});
