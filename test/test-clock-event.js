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

    assert(end - start >= 950);
  });

  it('should wait until system time', async function () {
    const clock = new Clock(ClockType.SYSTEM_TIME);
    const event = new rclnodejs.ClockEvent();
    const now = clock.now();
    const until = now.add(new Duration(1n, 0n)); // 1 second later

    const start = Date.now();
    await event.waitUntilSystem(clock, until.nanoseconds);
    const end = Date.now();

    assert(end - start >= 950);
  });

  it('should wait until ros time', async function () {
    // Note: This tests ROS_TIME clock when ROS time override is not enabled.
    // In this case, waitUntilRos() falls back to system time behavior.
    // For testing with active ROS time (TimeSource + published clock messages),
    // see test-clock-sleep.js "should work with ROSClock when ROS time is active"
    const clock = new Clock(ClockType.ROS_TIME);
    const event = new rclnodejs.ClockEvent();
    const now = clock.now();
    const until = now.add(new Duration(1n, 0n)); // 1 second later

    const start = Date.now();
    await event.waitUntilRos(clock, until.nanoseconds);
    const end = Date.now();

    assert(end - start >= 950);
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
