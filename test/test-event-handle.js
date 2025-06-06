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
const DistroUtils = require('../lib/distro.js');
const rclnodejs = require('../index.js');
const { SubscriptionEventCallbacks } = require('../lib/event_handler.js');
const { PublisherEventCallbacks } = require('../lib/event_handler.js');

describe('Event handle test suite prior to jazzy', function () {
  before(function () {
    if (DistroUtils.getDistroId() >= DistroUtils.getDistroId('jazzy')) {
      this.skip();
    }
  });

  it('Error expected when creating SubscriptionEventCallbacks', function () {
    assert.throws(() => {
      new SubscriptionEventCallbacks();
    }, /SubscriptionEventCallbacks is only available in ROS 2 Jazzy and later.$/);
  });

  it('Error expected when creating PublisherEventCallbacks', function () {
    assert.throws(() => {
      new PublisherEventCallbacks();
    }, /PublisherEventCallbacks is only available in ROS 2 Jazzy and later.$/);
  });
});

describe('Event handle test suite', function () {
  this.timeout(5 * 1000);
  let node;

  before(function () {
    if (DistroUtils.getDistroId() <= DistroUtils.getDistroId('humble')) {
      this.skip();
    }
  });

  beforeEach(async function () {
    await rclnodejs.init();
    const nodeName = 'test_event_handle';
    node = rclnodejs.createNode(nodeName);
    rclnodejs.spin(node);
  });

  afterEach(function () {
    rclnodejs.shutdown();
  });

  it('Test subscription constructor with callback', function () {
    const String = 'std_msgs/msg/String';
    let eventCallbacks = new SubscriptionEventCallbacks();
    let subscription = null;
    let expectedEventCount = 1;
    eventCallbacks.deadline = () => {};
    subscription = node.createSubscription(
      String,
      'topic',
      undefined,
      (msg) => {},
      eventCallbacks
    );
    assert.strictEqual(node._events.length, expectedEventCount);
    node.destroySubscription(subscription);

    eventCallbacks = new SubscriptionEventCallbacks();
    eventCallbacks.liveliness = () => {};
    eventCallbacks.deadline = () => {};
    expectedEventCount++;
    node.createSubscription(
      String,
      'topic',
      undefined,
      (msg) => {},
      eventCallbacks
    );
    assert.strictEqual(node._events.length, expectedEventCount);
  });

  it('Test subscription event apis', async function () {
    const String = 'std_msgs/msg/String';
    let eventCallbacks = new SubscriptionEventCallbacks();
    const deadlinePromise = new Promise((resolve) => {
      eventCallbacks.deadline = (deadline) => {
        assert.strictEqual(deadline.total_count, 0);
        assert.strictEqual(deadline.total_count_change, 0);
        resolve();
      };
    });
    const livelinessPromise = new Promise((resolve) => {
      eventCallbacks.liveliness = (liveliness) => {
        assert.strictEqual(liveliness.alive_count, 0);
        assert.strictEqual(liveliness.not_alive_count, 0);
        assert.strictEqual(liveliness.alive_count_change, 0);
        assert.strictEqual(liveliness.not_alive_count_change, 0);
        resolve();
      };
    });
    const incompatibleQosPromise = new Promise((resolve) => {
      eventCallbacks.incompatibleQos = (incompatibleQos) => {
        assert.strictEqual(incompatibleQos.total_count, 0);
        assert.strictEqual(incompatibleQos.total_count_change, 0);
        resolve();
      };
    });

    node.createSubscription(
      String,
      'topic',
      undefined,
      (msg) => {},
      eventCallbacks
    );
    node._events.forEach((event) => {
      event.takeData();
    });

    return Promise.all([
      deadlinePromise,
      livelinessPromise,
      incompatibleQosPromise,
    ]);
  });

  it('Test publisher event apis', async function () {
    const String = 'std_msgs/msg/String';
    let eventCallbacks = new PublisherEventCallbacks();
    const deadlinePromise = new Promise((resolve) => {
      eventCallbacks.deadline = (deadline) => {
        assert.strictEqual(deadline.total_count, 0);
        assert.strictEqual(deadline.total_count_change, 0);
        resolve();
      };
    });
    const livelinessPromise = new Promise((resolve) => {
      eventCallbacks.liveliness = (liveliness) => {
        assert.strictEqual(liveliness.total_count, 0);
        assert.strictEqual(liveliness.total_count_change, 0);
        resolve();
      };
    });
    const incompatibleQosPromise = new Promise((resolve) => {
      eventCallbacks.incompatibleQos = (incompatibleQos) => {
        assert.strictEqual(incompatibleQos.total_count, 0);
        assert.strictEqual(incompatibleQos.total_count_change, 0);
        resolve();
      };
    });

    node.createPublisher(String, 'topic', undefined, eventCallbacks);
    node._events.forEach((event) => {
      event.takeData();
    });

    return Promise.all([
      deadlinePromise,
      livelinessPromise,
      incompatibleQosPromise,
    ]);
  });

  it('Test subscription event matched', function (done) {
    const String = 'std_msgs/msg/String';
    let publisher = null;
    let publisherDestroyed = false;
    let eventCallbacks = new SubscriptionEventCallbacks();
    eventCallbacks.matched = (matched) => {
      if (!publisherDestroyed) {
        assert.strictEqual(matched.total_count, 1);
        assert.strictEqual(matched.total_count_change, 1);
        assert.strictEqual(matched.current_count, 1);
        assert.strictEqual(matched.current_count_change, 1);
        node.destroyPublisher(publisher);
        publisherDestroyed = true;
      } else {
        assert.strictEqual(matched.total_count, 1);
        assert.strictEqual(matched.total_count_change, 0);
        assert.strictEqual(matched.current_count, 0);
        assert.strictEqual(matched.current_count_change, -1);
        node.stop();
        done();
      }
    };
    node.createSubscription(
      String,
      'topic',
      undefined,
      (msg) => {},
      eventCallbacks
    );
    assert.strictEqual(node._events.length, 1);

    publisher = node.createPublisher(String, 'topic', undefined);
  });

  it('Test publisher event unmatched', function (done) {
    const String = 'std_msgs/msg/String';
    let subscription = null;
    let subscriptionDestroyed = false;
    let eventCallbacks = new PublisherEventCallbacks();
    eventCallbacks.matched = (matched) => {
      if (!subscriptionDestroyed) {
        assert.strictEqual(matched.total_count, 1);
        assert.strictEqual(matched.total_count_change, 1);
        assert.strictEqual(matched.current_count, 1);
        assert.strictEqual(matched.current_count_change, 1);
        node.destroySubscription(subscription);
        subscriptionDestroyed = true;
      } else {
        assert.strictEqual(matched.total_count, 1);
        assert.strictEqual(matched.total_count_change, 0);
        assert.strictEqual(matched.current_count, 0);
        assert.strictEqual(matched.current_count_change, -1);
        node.stop();
        done();
      }
    };

    node.createPublisher(String, 'topic', undefined, eventCallbacks);
    subscription = node.createSubscription(
      String,
      'topic',
      undefined,
      (msg) => {}
    );
  });
});
