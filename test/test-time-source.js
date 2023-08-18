// Copyright (c) 2018 Intel Corporation. All rights reserved.
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
const { Clock, Parameter, ParameterType, ROSClock, TimeSource, Time } =
  rclnodejs;
const int64 = require('int64-napi');

describe('rclnodejs TimeSource testing', function () {
  this.timeout(60 * 1000);
  let node;
  let timer;

  before(async function () {
    await rclnodejs.init();
  });

  this.beforeEach(() => {
    node = rclnodejs.createNode('TestTimeSource');
    rclnodejs.spin(node);
  });

  this.afterEach(() => {
    node.destroy();
    node = null;
  });

  after(function () {
    clearInterval(timer);
    rclnodejs.shutdown();
  });

  function publishClockMessage(node) {
    let pub = node.createPublisher('rosgraph_msgs/msg/Clock', '/clock');
    let count = 0;
    timer = setInterval(() => {
      pub.publish({ clock: { sec: count, nanosec: 0 } });
      count += 1;
    }, 1000);
  }

  it('Test attach clock to time source', function () {
    let timeSource = new TimeSource(node);
    timeSource.attachClock(new ROSClock());

    assert.throws(() => {
      timeSource.attachClock(new Clock());
    }, TypeError);
    assert.throws(() => {
      timeSource.attachClock(new Clock(Clock.ClockType.STEADY_TIME));
    }, TypeError);
  });

  it('Test not using sim time', function () {
    let timeSource = new TimeSource(node);
    let clock = new ROSClock();
    timeSource.attachClock(clock);

    let now = clock.now();
    let sysClock = new Clock(Clock.ClockType.SYSTEM_TIME);
    let sysNow = sysClock.now();
    assert.ok(int64.subtract(sysNow.nanoseconds, now.nanoseconds) < 1e9);

    publishClockMessage(node);
    assert.strictEqual(clock.isRosTimeActive, false);
    now = clock.now();
    sysNow = sysClock.now();
    assert.ok(int64.subtract(sysNow.nanoseconds, now.nanoseconds) < 1e9);

    assert.strictEqual(timeSource.isRosTimeActive, false);
    let clock2 = new ROSClock();
    clock2.isRosTimeActive = true;
    timeSource.attachClock(clock2);
    assert.strictEqual(clock2.isRosTimeActive, false);
    clearInterval(timer);
  });

  it('Test using sim time parameter', function (done) {
    // can not use default node.
    node.destroy();

    // create node with use_sim_time parameter true
    const options = new rclnodejs.NodeOptions();
    options.parameterOverrides.push(
      new rclnodejs.Parameter(
        'use_sim_time',
        rclnodejs.ParameterType.PARAMETER_BOOL,
        true
      )
    );
    node = rclnodejs.createNode(
      'TestNode1',
      '',
      rclnodejs.Context.defaultContext(),
      options
    );
    rclnodejs.spin(node);

    assert.ok(node.hasParameter('use_sim_time'));
    assert.ok(node.getParameter('use_sim_time').value);
    let timeSource = new TimeSource(node);
    let clock = new ROSClock();
    timeSource.attachClock(clock);

    assert.strictEqual(timeSource.isRosTimeActive, true);
    assert.strictEqual(clock.isRosTimeActive, true);

    assert.ok(timeSource._clockSubscription);
    assert.strictEqual(
      clock.now().eq(new Time(0, 0, Clock.ClockType.ROS_TIME)),
      true
    );

    publishClockMessage(node);
    setTimeout(() => {
      assert.ok(clock.now().gt(new Time(0, 0, Clock.ClockType.ROS_TIME)));
      assert.ok(clock.now().lte(new Time(5, 0, Clock.ClockType.ROS_TIME)));

      let clock2 = new ROSClock();
      timeSource.attachClock(clock2);
      assert.ok(clock2.now().gt(new Time(0, 0, Clock.ClockType.ROS_TIME)));
      assert.ok(clock2.now().lte(new Time(5, 0, Clock.ClockType.ROS_TIME)));

      timeSource.detachNode();
      let node2 = rclnodejs.createNode(
        'TestTimeSource1',
        '',
        rclnodejs.Context.defaultContext(),
        options
      );
      timeSource.attachNode(node2);
      assert.strictEqual(timeSource._node, node2);
      assert.ok(timeSource._clockSubscription);
      clearInterval(timer);
      done();
    }, 3000);
  });
});
