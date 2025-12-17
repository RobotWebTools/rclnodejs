'use strict';

const assert = require('assert');
const rclnodejs = require('../index.js');
const { Clock, ClockType, Duration } = rclnodejs;

describe('ClockEvent', function () {
  let node;
  this.timeout(10000);

  before(async function () {
    await rclnodejs.init();
    node = rclnodejs.createNode('test_clock_event_node');
    rclnodejs.spin(node);
  });

  after(function () {
    rclnodejs.shutdown();
  });

  it('should wait until steady time', async function () {
    const clock = new Clock(ClockType.STEADY_TIME);
    const event = new rclnodejs.ClockEvent();
    const now = clock.now();
    const until = now.add(new Duration(1n, 0n)); // 1 second later

    const start = Date.now();
    await event.waitUntilSteady(clock, until.nanoseconds);
    const end = Date.now();

    assert(end - start >= 1000);
  });

  it('should wait until system time', async function () {
    const clock = new Clock(ClockType.SYSTEM_TIME);
    const event = new rclnodejs.ClockEvent();
    const now = clock.now();
    const until = now.add(new Duration(1n, 0n)); // 1 second later

    const start = Date.now();
    await event.waitUntilSystem(clock, until.nanoseconds);
    const end = Date.now();

    assert(end - start >= 1000);
  });

  it('should wait until ros time', async function () {
    const clock = new Clock(ClockType.ROS_TIME);
    const event = new rclnodejs.ClockEvent();
    const now = clock.now();
    const until = now.add(new Duration(1n, 0n)); // 1 second later

    const start = Date.now();
    await event.waitUntilRos(clock, until.nanoseconds);
    const end = Date.now();

    assert(end - start >= 1000);
  });

  it('should set and clear event', function () {
    const event = new rclnodejs.ClockEvent();
    assert.strictEqual(event.isSet(), false);
    event.set();
    assert.strictEqual(event.isSet(), true);
    event.clear();
    assert.strictEqual(event.isSet(), false);
  });
});
