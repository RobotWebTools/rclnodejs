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
const sinon = require('sinon');
const DistroUtils = require('../lib/distro.js');
const rclnodejs = require('../index.js');
const rclnodejsBinding = require('../lib/native_loader.js');
const {
  SubscriptionEventCallbacks,
  PublisherEventCallbacks,
  PublisherEventType,
  SubscriptionEventType,
  isPublisherEventTypeSupported,
  isSubscriptionEventTypeSupported,
} = require('../lib/event_handler.js');

describe('Event handle test suite prior to jazzy', function () {
  before(function () {
    if (DistroUtils.getDistroId() >= DistroUtils.getDistroId('jazzy')) {
      this.skip();
    }
  });

  it('Error expected when creating SubscriptionEventCallbacks', function () {
    assert.throws(() => {
      new SubscriptionEventCallbacks();
    }, /SubscriptionEventCallbacks is only available in ROS 2 Jazzy and later/);
  });

  it('Error expected when creating PublisherEventCallbacks', function () {
    assert.throws(() => {
      new PublisherEventCallbacks();
    }, /PublisherEventCallbacks is only available in ROS 2 Jazzy and later/);
  });
});

describe('Event type is supported - native binding unavailable', function () {
  before(function () {
    if (
      typeof rclnodejsBinding.isPublisherEventTypeSupported === 'function' &&
      typeof rclnodejsBinding.isSubscriptionEventTypeSupported === 'function'
    ) {
      this.skip();
    }
  });

  it('isPublisherEventTypeSupported throws when native binding is missing', function () {
    assert.throws(() => {
      isPublisherEventTypeSupported(PublisherEventType.PUBLISHER_MATCHED);
    }, /isPublisherEventTypeSupported is only available in ROS 2 Rolling and later/);
  });

  it('isSubscriptionEventTypeSupported throws when native binding is missing', function () {
    assert.throws(() => {
      isSubscriptionEventTypeSupported(
        SubscriptionEventType.SUBSCRIPTION_MATCHED
      );
    }, /isSubscriptionEventTypeSupported is only available in ROS 2 Rolling and later/);
  });
});

describe('Event type is supported - native binding available', function () {
  before(function () {
    if (
      typeof rclnodejsBinding.isPublisherEventTypeSupported !== 'function' ||
      typeof rclnodejsBinding.isSubscriptionEventTypeSupported !== 'function'
    ) {
      this.skip();
    }
  });

  it('isPublisherEventTypeSupported returns a boolean for every event type', function () {
    for (const eventType of Object.values(PublisherEventType)) {
      const result = isPublisherEventTypeSupported(eventType);
      assert.strictEqual(typeof result, 'boolean');
    }
  });

  it('isSubscriptionEventTypeSupported returns a boolean for every event type', function () {
    for (const eventType of Object.values(SubscriptionEventType)) {
      const result = isSubscriptionEventTypeSupported(eventType);
      assert.strictEqual(typeof result, 'boolean');
    }
  });

  it('MATCHED events are reported as supported across RMW implementations', function () {
    assert.strictEqual(
      isPublisherEventTypeSupported(PublisherEventType.PUBLISHER_MATCHED),
      true
    );
    assert.strictEqual(
      isSubscriptionEventTypeSupported(
        SubscriptionEventType.SUBSCRIPTION_MATCHED
      ),
      true
    );
  });

  it('isPublisherEventTypeSupported rejects invalid event types', function () {
    assert.throws(() => {
      isPublisherEventTypeSupported(-1);
    }, /Value '-1' for 'eventType' is out of range: one of PublisherEventType values/);
    assert.throws(() => {
      isPublisherEventTypeSupported('matched');
    }, /Invalid type for 'eventType': expected number, got string/);
  });

  it('isSubscriptionEventTypeSupported rejects invalid event types', function () {
    assert.throws(() => {
      isSubscriptionEventTypeSupported(999);
    }, /Value '999' for 'eventType' is out of range: one of SubscriptionEventType values/);
    assert.throws(() => {
      isSubscriptionEventTypeSupported(undefined);
    }, /Invalid type for 'eventType': expected number, got undefined/);
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

describe('EventHandler unit testing (Mocks)', function () {
  let sandbox;
  let addedProps = [];

  function stubOptional(obj, method) {
    if (!obj[method]) {
      obj[method] = () => {};
      addedProps.push({ obj, method });
    }
    return sandbox.stub(obj, method);
  }

  beforeEach(function () {
    sandbox = sinon.createSandbox();
    addedProps = [];
  });

  afterEach(function () {
    sandbox.restore();
    addedProps.forEach(({ obj, method }) => {
      delete obj[method];
    });
    addedProps = [];
  });

  describe('PublisherEventCallbacks', function () {
    it('throws on unsupported distro', function () {
      // Mock DistroUtils to return old version
      sandbox.stub(DistroUtils, 'getDistroId').callsFake((val) => {
        if (val === 'jazzy') return 10;
        return 5;
      });

      assert.throws(() => {
        new PublisherEventCallbacks();
      }, /only available in ROS 2 Jazzy/);
    });

    it('constructs on supported distro', function () {
      sandbox.stub(DistroUtils, 'getDistroId').callsFake((val) => {
        if (val === 'jazzy') return 10;
        return 11;
      });

      const cb = new PublisherEventCallbacks();
      assert.ok(cb);
      assert.deepStrictEqual(cb.eventHandlers, []);
    });

    it('getters and setters work', function () {
      sandbox.stub(DistroUtils, 'getDistroId').callsFake((val) => {
        if (val === 'jazzy') return 10;
        return 11;
      });

      const cb = new PublisherEventCallbacks();
      const fn = () => {};

      cb.deadline = fn;
      assert.strictEqual(cb.deadline, fn);

      cb.incompatibleQos = fn;
      assert.strictEqual(cb.incompatibleQos, fn);

      cb.liveliness = fn;
      assert.strictEqual(cb.liveliness, fn);

      cb.incompatibleType = fn;
      assert.strictEqual(cb.incompatibleType, fn);

      cb.matched = fn;
      assert.strictEqual(cb.matched, fn);
    });

    it('createEventHandlers creates handles', function () {
      sandbox.stub(DistroUtils, 'getDistroId').callsFake((val) => {
        if (val === 'jazzy') return 10;
        return 11;
      });

      const createStub = stubOptional(
        rclnodejsBinding,
        'createPublisherEventHandle'
      ).returns('mock-event-handle');

      const cb = new PublisherEventCallbacks();
      cb.deadline = () => {};

      const handlers = cb.createEventHandlers('pub-handle');

      assert.strictEqual(handlers.length, 1);
      assert.strictEqual(createStub.calledOnce, true);
      assert.strictEqual(
        createStub.firstCall.args[1],
        PublisherEventType.PUBLISHER_OFFERED_DEADLINE_MISSED
      );
    });
  });

  describe('SubscriptionEventCallbacks', function () {
    it('throws on unsupported distro', function () {
      sandbox.stub(DistroUtils, 'getDistroId').callsFake((val) => {
        if (val === 'jazzy') return 10;
        return 5;
      });

      assert.throws(() => {
        new SubscriptionEventCallbacks();
      }, /only available in ROS 2 Jazzy/);
    });

    it('constructs on supported distro', function () {
      sandbox.stub(DistroUtils, 'getDistroId').callsFake((val) => {
        if (val === 'jazzy') return 10;
        return 11;
      });

      const cb = new SubscriptionEventCallbacks();
      assert.ok(cb);
    });

    it('getters and setters work', function () {
      sandbox.stub(DistroUtils, 'getDistroId').callsFake((val) => {
        if (val === 'jazzy') return 10;
        return 11;
      });

      const cb = new SubscriptionEventCallbacks();
      const fn = () => {};

      cb.messageLost = fn;
      assert.strictEqual(cb.messageLost, fn);
    });

    it('createEventHandlers creates handles', function () {
      sandbox.stub(DistroUtils, 'getDistroId').callsFake((val) => {
        if (val === 'jazzy') return 10;
        return 11;
      });

      const createStub = stubOptional(
        rclnodejsBinding,
        'createSubscriptionEventHandle'
      ).returns('mock-sub-event-handle');

      const cb = new SubscriptionEventCallbacks();
      cb.messageLost = () => {};

      const handlers = cb.createEventHandlers('sub-handle');

      assert.strictEqual(handlers.length, 1);
      assert.strictEqual(
        createStub.calledWith(
          'sub-handle',
          SubscriptionEventType.SUBSCRIPTION_MESSAGE_LOST
        ),
        true
      );
    });
  });

  describe('EventHandler interaction', function () {
    it('takeData calls callback', function () {
      sandbox.stub(DistroUtils, 'getDistroId').callsFake(() => 999);
      const takeEventStub = stubOptional(rclnodejsBinding, 'takeEvent').returns(
        { count: 1 }
      );

      const cb = new PublisherEventCallbacks();
      const spy = sinon.spy();
      cb.deadline = spy;

      stubOptional(rclnodejsBinding, 'createPublisherEventHandle').returns(
        'handle'
      );
      cb.createEventHandlers('pub');

      const handler = cb.eventHandlers[0];
      handler.takeData();

      assert.ok(takeEventStub.called);
      assert.ok(spy.calledWith({ count: 1 }));
    });
  });
});
