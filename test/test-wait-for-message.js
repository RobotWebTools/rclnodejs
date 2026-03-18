// Copyright (c) 2026, The Robot Web Tools Contributors
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

describe('waitForMessage tests', function () {
  this.timeout(60 * 1000);

  let node;

  before(function () {
    return rclnodejs.init();
  });

  after(function () {
    rclnodejs.shutdown();
  });

  beforeEach(function () {
    node = rclnodejs.createNode('wait_for_message_test_node');
    node.spin();
  });

  afterEach(function () {
    node.stop();
    node.destroy();
  });

  it('should receive a message', async function () {
    const publisher = node.createPublisher(
      'std_msgs/msg/String',
      'wfm_test_topic_1'
    );

    // Publish after a short delay
    setTimeout(() => {
      publisher.publish('hello waitForMessage');
    }, 200);

    const msg = await rclnodejs.waitForMessage(
      'std_msgs/msg/String',
      node,
      'wfm_test_topic_1',
      { timeout: 5000 }
    );

    assert.strictEqual(msg.data, 'hello waitForMessage');
  });

  it('should timeout when no message arrives', async function () {
    await assert.rejects(
      () =>
        rclnodejs.waitForMessage(
          'std_msgs/msg/String',
          node,
          'wfm_nonexistent_topic',
          { timeout: 500 }
        ),
      { message: /timed out/ }
    );
  });

  it('should receive only the first message', async function () {
    const publisher = node.createPublisher(
      'std_msgs/msg/String',
      'wfm_test_topic_2'
    );

    let receiveCount = 0;

    setTimeout(() => {
      publisher.publish('first');
      publisher.publish('second');
      publisher.publish('third');
    }, 200);

    const msg = await rclnodejs.waitForMessage(
      'std_msgs/msg/String',
      node,
      'wfm_test_topic_2',
      { timeout: 5000 }
    );

    assert.strictEqual(msg.data, 'first');
  });

  it('should work with different message types', async function () {
    const publisher = node.createPublisher(
      'std_msgs/msg/Int32',
      'wfm_test_topic_3'
    );

    setTimeout(() => {
      publisher.publish({ data: 42 });
    }, 200);

    const msg = await rclnodejs.waitForMessage(
      'std_msgs/msg/Int32',
      node,
      'wfm_test_topic_3',
      { timeout: 5000 }
    );

    assert.strictEqual(msg.data, 42);
  });

  it('should wait indefinitely when no timeout is specified', async function () {
    const publisher = node.createPublisher(
      'std_msgs/msg/String',
      'wfm_test_topic_4'
    );

    // Publish after a delay — should still be caught without timeout
    setTimeout(() => {
      publisher.publish('delayed message');
    }, 500);

    const msg = await rclnodejs.waitForMessage(
      'std_msgs/msg/String',
      node,
      'wfm_test_topic_4'
    );

    assert.strictEqual(msg.data, 'delayed message');
  });

  it('should clean up subscription after receiving', async function () {
    const publisher = node.createPublisher(
      'std_msgs/msg/String',
      'wfm_test_topic_5'
    );

    const subCountBefore = node._subscriptions.length;

    setTimeout(() => {
      publisher.publish('cleanup test');
    }, 200);

    await rclnodejs.waitForMessage(
      'std_msgs/msg/String',
      node,
      'wfm_test_topic_5',
      { timeout: 5000 }
    );

    // Subscription should be cleaned up
    assert.strictEqual(node._subscriptions.length, subCountBefore);
  });

  it('should clean up subscription on timeout', async function () {
    const subCountBefore = node._subscriptions.length;

    await assert.rejects(() =>
      rclnodejs.waitForMessage(
        'std_msgs/msg/String',
        node,
        'wfm_timeout_cleanup_topic',
        { timeout: 300 }
      )
    );

    // Subscription should be cleaned up even on timeout
    assert.strictEqual(node._subscriptions.length, subCountBefore);
  });
});
