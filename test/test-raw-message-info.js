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

import assert from 'assert';
import rclnodejs from '../index.js';

describe('Raw subscription MessageInfo tests', function () {
  this.timeout(60 * 1000);

  let node;

  before(function () {
    return rclnodejs.init();
  });

  after(function () {
    rclnodejs.shutdown();
  });

  beforeEach(function () {
    node = rclnodejs.createNode('raw_message_info_test_node');
  });

  afterEach(function () {
    node.destroy();
  });

  it('should receive MessageInfo with a raw subscription when callback has 2 parameters', function (done) {
    const publisher = node.createPublisher(
      'std_msgs/msg/String',
      'raw_mi_test_topic_1'
    );

    node.createSubscription(
      'std_msgs/msg/String',
      'raw_mi_test_topic_1',
      { isRaw: true },
      (buffer, messageInfo) => {
        assert.ok(Buffer.isBuffer(buffer), 'raw message should be a Buffer');
        assert.ok(messageInfo, 'messageInfo should be provided');
        assert.ok(
          typeof messageInfo.sourceTimestamp === 'bigint',
          'sourceTimestamp should be a bigint'
        );
        assert.ok(
          typeof messageInfo.receivedTimestamp === 'bigint',
          'receivedTimestamp should be a bigint'
        );
        assert.ok(
          messageInfo.receivedTimestamp >= messageInfo.sourceTimestamp,
          'receivedTimestamp should be >= sourceTimestamp'
        );
        assert.ok(
          Buffer.isBuffer(messageInfo.publisherGid),
          'publisherGid should be a Buffer'
        );
        assert.ok(
          messageInfo.publisherGid.length > 0,
          'publisherGid should not be empty'
        );
        done();
      }
    );

    rclnodejs.spin(node);

    setTimeout(() => {
      publisher.publish('Hello raw MessageInfo');
    }, 200);
  });

  it('should deliver only the Buffer when a raw callback has 1 parameter', function (done) {
    const publisher = node.createPublisher(
      'std_msgs/msg/String',
      'raw_mi_test_topic_2'
    );

    node.createSubscription(
      'std_msgs/msg/String',
      'raw_mi_test_topic_2',
      { isRaw: true },
      function (buffer) {
        assert.ok(Buffer.isBuffer(buffer), 'raw message should be a Buffer');
        assert.strictEqual(
          arguments.length,
          1,
          'no MessageInfo when not requested'
        );
        done();
      }
    );

    rclnodejs.spin(node);

    setTimeout(() => {
      publisher.publish('Hello raw no info');
    }, 200);
  });

  it('should distinguish publishers sharing a topic by publisherGid', function (done) {
    const topic = 'raw_mi_test_topic_3';
    const first = node.createPublisher('std_msgs/msg/String', topic);
    const second = node.createPublisher('std_msgs/msg/String', topic);
    const gids = new Set();
    let finished = false;

    node.createSubscription(
      'std_msgs/msg/String',
      topic,
      { isRaw: true },
      (buffer, messageInfo) => {
        if (finished) return;
        gids.add(messageInfo.publisherGid.toString('hex'));
        if (gids.size === 2) {
          finished = true;
          clearInterval(timer);
          done();
        }
      }
    );

    rclnodejs.spin(node);

    const timer = setInterval(() => {
      first.publish('from first');
      second.publish('from second');
    }, 100);
  });
});
