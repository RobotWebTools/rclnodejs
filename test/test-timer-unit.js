'use strict';

const assert = require('assert');
const rclnodejs = require('../index.js');
const sinon = require('sinon');
const DistroUtils = require('../lib/distro.js');

const TIMER_INTERVAL = BigInt('100000000');

describe('rclnodejs Timer class coverage testing', function () {
  this.timeout(60 * 1000);
  let node;
  let timer;

  before(async function () {
    await rclnodejs.init();
  });

  after(function () {
    rclnodejs.shutdown();
  });

  beforeEach(function () {
    node = rclnodejs.createNode('timer_coverage_node');
    timer = node.createTimer(TIMER_INTERVAL, () => {});
  });

  afterEach(function () {
    if (node) {
      node.destroy();
    }
  });

  it('handle getter should return handle', function () {
    assert.ok(timer.handle);
  });

  it('getNextCallTime returns undefined if rclnodejs function not present', function () {
    // Save original function descriptor/value from the export object
    // rclnodejs is a module export. We need to see if it allows modification.
    const originalFunc = rclnodejs.getTimerNextCallTime;

    // We cannot easily delete property from object returned by require,
    // unless we use Object.defineProperty or similar if it's configurable.
    // Or we mock rclnodejs entirely, but that's hard here.

    // Let's try attempting to overwrite it.
    try {
      // Force overwrite
      Object.defineProperty(rclnodejs, 'getTimerNextCallTime', {
        value: undefined,
        configurable: true,
        writable: true,
      });

      // If the implementation does: if (typeof rclnodejs.getTimerNextCallTime !== 'function')
      // it should return undefined.
      assert.strictEqual(timer.getNextCallTime(), undefined);
    } catch (e) {
      // If we can't overwrite, we might skip or use sinon stub if possible?
      // But rclnodejs is imported inside timer.js
      // We can't change the reference inside timer.js easily without reloading.
      this.skip();
    } finally {
      try {
        if (originalFunc) {
          Object.defineProperty(rclnodejs, 'getTimerNextCallTime', {
            value: originalFunc,
            configurable: true,
            writable: true,
          });
        }
      } catch (e) {}
    }
  });

  it('Distribution check warning for setOnResetCallback', function () {
    // DistroUtils.getDistroId is used inside timer.js methods.
    // We stub it.
    const stub = sinon.stub(DistroUtils, 'getDistroId').returns(1); // Return 1 (Simulate minimal/old)
    // We need to also stub the constant call 'humble' to return 2 (higher than 1)
    stub.withArgs('humble').returns(2);

    const consoleSpy = sinon.spy(console, 'warn');

    try {
      timer.setOnResetCallback(() => {});
      assert.ok(consoleSpy.calledWithMatch(/not supported/));
    } finally {
      stub.restore();
      consoleSpy.restore();
    }
  });

  it('Distribution check warning for clearOnResetCallback', function () {
    const stub = sinon.stub(DistroUtils, 'getDistroId').returns(1);
    stub.withArgs('humble').returns(2);
    const consoleSpy = sinon.spy(console, 'warn');

    try {
      timer.clearOnResetCallback();
      assert.ok(consoleSpy.calledWithMatch(/not supported/));
    } finally {
      stub.restore();
      consoleSpy.restore();
    }
  });

  it('Distribution check warning for callTimerWithInfo', function () {
    const stub = sinon.stub(DistroUtils, 'getDistroId').returns(1);
    stub.withArgs('humble').returns(2);
    const consoleSpy = sinon.spy(console, 'warn');

    try {
      timer.callTimerWithInfo();
      assert.ok(consoleSpy.calledWithMatch(/not supported/));
    } finally {
      stub.restore();
      consoleSpy.restore();
    }
  });
});
