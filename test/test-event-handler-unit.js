'use strict';

const assert = require('assert');
const sinon = require('sinon');
const DistroUtils = require('../lib/distro.js');
const rclnodejsBinding = require('../lib/native_loader.js'); // This is what event_handler requires
const {
  PublisherEventCallbacks,
  SubscriptionEventCallbacks,
  PublisherEventType,
  SubscriptionEventType,
} = require('../lib/event_handler.js');

describe('EventHandler unit testing', function () {
  let sandbox;

  beforeEach(function () {
    sandbox = sinon.createSandbox();
  });

  afterEach(function () {
    sandbox.restore();
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

      const createStub = sandbox
        .stub(rclnodejsBinding, 'createPublisherEventHandle')
        .returns('mock-event-handle');

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

      const createStub = sandbox
        .stub(rclnodejsBinding, 'createSubscriptionEventHandle')
        .returns('mock-sub-event-handle');

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
      const takeEventStub = sandbox
        .stub(rclnodejsBinding, 'takeEvent')
        .returns({ count: 1 });

      const cb = new PublisherEventCallbacks();
      const spy = sinon.spy();
      cb.deadline = spy;

      sandbox
        .stub(rclnodejsBinding, 'createPublisherEventHandle')
        .returns('handle');
      cb.createEventHandlers('pub');

      const handler = cb.eventHandlers[0];
      handler.takeData();

      assert.ok(takeEventStub.called);
      assert.ok(spy.calledWith({ count: 1 }));
    });
  });
});
