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

describe('MessageInfo tests', function () {
  this.timeout(60 * 1000);

  let node;

  before(function () {
    return rclnodejs.init();
  });

  after(function () {
    rclnodejs.shutdown();
  });

  beforeEach(function () {
    node = rclnodejs.createNode('message_info_test_node');
  });

  afterEach(function () {
    node.destroy();
  });

  it('should receive MessageInfo when callback has 2 parameters', function (done) {
    const publisher = node.createPublisher(
      'std_msgs/msg/String',
      'mi_test_topic_1'
    );

    node.createSubscription(
      'std_msgs/msg/String',
      'mi_test_topic_1',
      (msg, messageInfo) => {
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
          typeof messageInfo.publicationSequenceNumber === 'bigint',
          'publicationSequenceNumber should be a bigint'
        );
        assert.ok(
          typeof messageInfo.receptionSequenceNumber === 'bigint',
          'receptionSequenceNumber should be a bigint'
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
      publisher.publish('Hello MessageInfo');
    }, 200);
  });

  it('should NOT receive MessageInfo when callback has 1 parameter', function (done) {
    const publisher = node.createPublisher(
      'std_msgs/msg/String',
      'mi_test_topic_2'
    );

    node.createSubscription('std_msgs/msg/String', 'mi_test_topic_2', (msg) => {
      assert.strictEqual(typeof msg, 'object');
      assert.strictEqual(msg.data, 'Hello no info');
      // msg should be the message, not MessageInfo
      assert.ok(!msg.sourceTimestamp, 'should not have sourceTimestamp');
      done();
    });

    rclnodejs.spin(node);

    setTimeout(() => {
      publisher.publish('Hello no info');
    }, 200);
  });

  it('sourceTimestamp should be a positive value', function (done) {
    const publisher = node.createPublisher(
      'std_msgs/msg/String',
      'mi_test_topic_3'
    );

    node.createSubscription(
      'std_msgs/msg/String',
      'mi_test_topic_3',
      (msg, messageInfo) => {
        assert.ok(
          messageInfo.sourceTimestamp > 0n,
          'sourceTimestamp should be positive'
        );
        assert.ok(
          messageInfo.receivedTimestamp > 0n,
          'receivedTimestamp should be positive'
        );
        done();
      }
    );

    rclnodejs.spin(node);

    setTimeout(() => {
      publisher.publish('Timestamp test');
    }, 200);
  });

  it('receivedTimestamp should be >= sourceTimestamp', function (done) {
    const publisher = node.createPublisher(
      'std_msgs/msg/String',
      'mi_test_topic_4'
    );

    node.createSubscription(
      'std_msgs/msg/String',
      'mi_test_topic_4',
      (msg, messageInfo) => {
        assert.ok(
          messageInfo.receivedTimestamp >= messageInfo.sourceTimestamp,
          'receivedTimestamp should be >= sourceTimestamp'
        );
        done();
      }
    );

    rclnodejs.spin(node);

    setTimeout(() => {
      publisher.publish('Ordering test');
    }, 200);
  });

  it('toPlainObject should return all fields', function (done) {
    const publisher = node.createPublisher(
      'std_msgs/msg/String',
      'mi_test_topic_5'
    );

    node.createSubscription(
      'std_msgs/msg/String',
      'mi_test_topic_5',
      (msg, messageInfo) => {
        const plain = messageInfo.toPlainObject();
        assert.ok(plain.sourceTimestamp !== undefined);
        assert.ok(plain.receivedTimestamp !== undefined);
        assert.ok(plain.publicationSequenceNumber !== undefined);
        assert.ok(plain.receptionSequenceNumber !== undefined);
        assert.ok(plain.publisherGid !== undefined);
        done();
      }
    );

    rclnodejs.spin(node);

    setTimeout(() => {
      publisher.publish('Plain object test');
    }, 200);
  });

  it('MessageInfo class should be exported', function () {
    assert.ok(rclnodejs.MessageInfo);
  });
});
