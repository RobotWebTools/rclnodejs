// Copyright (c) 2025, The Robot Web Tools Contributors
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

describe('rclnodejs graph test suite', function () {
  this.timeout(60 * 1000);

  beforeEach(function () {
    return rclnodejs.init();
  });

  afterEach(function () {
    rclnodejs.shutdown();
  });

  it('Get publishers info by topic', function () {
    const node = rclnodejs.createNode('publisher_node', '/my_ns');
    assert.deepStrictEqual(
      0,
      node.getPublishersInfoByTopic('/my_ns/topic', false).length
    );
    const String = 'std_msgs/msg/String';
    node.createPublisher(String, 'topic');
    const publishers = node.getPublishersInfoByTopic('/my_ns/topic', false);
    assert.strictEqual(publishers.length, 1);
    assert.strictEqual(publishers[0].node_namespace, '/my_ns');
    assert.strictEqual(publishers[0].node_name, 'publisher_node');
    assert.strictEqual(publishers[0].topic_type, String);
  });

  it('Get subscriptions info by topic', function () {
    const node = rclnodejs.createNode('subscription_node', '/my_ns');
    assert.deepStrictEqual(
      0,
      node.getSubscriptionsInfoByTopic('/my_ns/topic', false).length
    );
    const String = 'std_msgs/msg/String';
    node.createSubscription(String, 'topic', (msg) => {});
    const subscriptions = node.getSubscriptionsInfoByTopic(
      '/my_ns/topic',
      false
    );
    assert.strictEqual(subscriptions.length, 1);
    assert.strictEqual(subscriptions[0].node_namespace, '/my_ns');
    assert.strictEqual(subscriptions[0].node_name, 'subscription_node');
    assert.strictEqual(subscriptions[0].topic_type, String);
  });
});
