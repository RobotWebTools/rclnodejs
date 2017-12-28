// Copyright (c) 2017 Intel Corporation. All rights reserved.
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

function isSameSecond(t, c) {
  return Math.floor(t / 1000) == c.sec;
}

function isSameMilliSecond(t, c, tolerance = 10) {
  t = t % 1000;
  c = Math.floor(c.nanosec / 1000 / 1000);
  if (t > c) {
    return t - c < tolerance;
  }
  return c - t < tolerance;
}

describe('rclnodejs Time/Clock testing', function() {
  this.timeout(60 * 1000);

  before(function() {
    return rclnodejs.init();
  });

  after(function() {
    rclnodejs.shutdown();
  });

  it('Create system/ros/steady clocks', function() {
    const c1 = new rclnodejs.Time('system-clock');
    const c2 = new rclnodejs.Time('ros-clock');
    const c3 = new rclnodejs.Time('steady-clock');
  });

  it('Get system/ros/steady clock time', function() {
    const c1 = new rclnodejs.Time('system-clock');
    const c2 = new rclnodejs.Time('ros-clock');
    const c3 = new rclnodejs.Time('steady-clock');

    assert(c1.now() instanceof Object, 'Should be able to get system-clock time');
    assert(c2.now() instanceof Object, 'Should be able to get ros-clock time');
    // assert(c3.now() instanceof Object, 'Should be able to get steady-clock time');
  });

  it('Get system/ros/steady clock time directly', function() {
    assert(rclnodejs.systemClockGetNow() instanceof Object, 'rclnodejs.systemClockGetNow() should work');
    assert(rclnodejs.rosClockGetNow() instanceof Object, 'rclnodejs.rosClockGetNow() should work');
    // assert(rclnodejs.steadyClockGetNow() instanceof Object, 'rclnodejs.steadyClockGetNow() should work');
  });

  it('Verify system clock time', function() {
    const c = new rclnodejs.Time('system-clock');
    const t1 = new Date().getTime();
    const t2 = c.now();
    assert(isSameSecond(t1, t2), 'Should be in the same second');
    assert(isSameMilliSecond(t1, t2), 'Should be approx the same millisecond');
  });

  it('Verify system clock Time.diff()', function() {
    const c = new rclnodejs.Time('system-clock');
    const t1 = c.now();
    const t2 = c.now();
    const delta = rclnodejs.Time.diff(t1, t2);
    assert(delta.sec == 0, 'Should be in the same second');
    assert(delta.nanosec >= 1000, 'Should elapse at least 1 microsecond');
  });

  /* eslint-disable key-spacing*/
  [
    // Positive diff
    {
      start:    {sec: 0, nanosec: 0, type: 'ros-clock'},
      finish:   {sec: 0, nanosec: 0, type: 'ros-clock'},
      expected: {sec: 0, nanosec: 0},
    },
    {
      start:    {sec: 0, nanosec: 0, type: 'ros-clock'},
      finish:   {sec: 1, nanosec: 0, type: 'ros-clock'},
      expected: {sec: 1, nanosec: 0},
    },
    {
      start:    {sec: 0, nanosec: 0, type: 'ros-clock'},
      finish:   {sec: 2, nanosec: 0, type: 'ros-clock'},
      expected: {sec: 2, nanosec: 0},
    },
    {
      start:    {sec: 0, nanosec: 1000, type: 'ros-clock'},
      finish:   {sec: 0, nanosec: 2000, type: 'ros-clock'},
      expected: {sec: 0, nanosec: 1000},
    },
    {
      start:    {sec: 0, nanosec: 2000, type: 'ros-clock'},
      finish:   {sec: 0, nanosec: 1000, type: 'ros-clock'},
      expected: {sec: 0, nanosec: -1000},
    },
    {
      start:    {sec: 1514423497, nanosec: 204, type: 'ros-clock'},
      finish:   {sec: 1514423497, nanosec: 804, type: 'ros-clock'},
      expected: {sec: 0, nanosec: 600},
    },
    {
      start:    {sec: 1514423496, nanosec: 204, type: 'ros-clock'},
      finish:   {sec: 1514423497, nanosec: 804, type: 'ros-clock'},
      expected: {sec: 1, nanosec: 600},
    },
    {
      start:    {sec: 1514423496, nanosec: 204, type: 'ros-clock'},
      finish:   {sec: 1514423497, nanosec: 804, type: 'ros-clock'},
      expected: {sec: 1, nanosec: 600},
    },
    {
      start:    {sec: 0, nanosec: 0, type: 'ros-clock'},
      finish:   {sec: 2, nanosec: 147483647, type: 'ros-clock'},
      expected: {sec: 2, nanosec: 147483647},
    },
    {
      start:    {sec: 0, nanosec: 500000000, type: 'ros-clock'},
      finish:   {sec: 2, nanosec: 0, type: 'ros-clock'},
      expected: {sec: 1, nanosec: 500000000},
    },

    // Negative diff
    {
      start:    {sec: 1, nanosec: 0, type: 'ros-clock'},
      finish:   {sec: 0, nanosec: 0, type: 'ros-clock'},
      expected: {sec: -1, nanosec: 0},
    },
    {
      start:    {sec: 2, nanosec: 0, type: 'ros-clock'},
      finish:   {sec: 0, nanosec: 0, type: 'ros-clock'},
      expected: {sec: -2, nanosec: 0},
    },
    {
      start:    {sec: 1, nanosec: 3000, type: 'ros-clock'},
      finish:   {sec: 0, nanosec: 0, type: 'ros-clock'},
      expected: {sec: -1, nanosec: -3000},
    },
    {
      start:    {sec: 1514423496, nanosec: 904, type: 'ros-clock'},
      finish:   {sec: 1514423496, nanosec: 804, type: 'ros-clock'},
      expected: {sec: 0, nanosec: -100},
    },
    {
      start:    {sec: 1514423497, nanosec: 904, type: 'ros-clock'},
      finish:   {sec: 1514423496, nanosec: 804, type: 'ros-clock'},
      expected: {sec: -1, nanosec: -100},
    },
    {
      start:    {sec: 1514423497, nanosec: 0, type: 'ros-clock'},
      finish:   {sec: 1514423496, nanosec: 600000000, type: 'ros-clock'},
      expected: {sec: 0, nanosec: -400000000},
    },

    // The following cases would fail, waiting for https://github.com/ros2/rcl/issues/204
    // {
    //   start:    {sec: 0, nanosec: 0, type: 'ros-clock'},
    //   finish:   {sec: 2, nanosec: 147483648, type: 'ros-clock'},
    //   expected: {sec: 2, nanosec: 147483648},
    // },
    // {
    //   start:    {sec: 1514423496, nanosec: 0, type: 'ros-clock'},
    //   finish:   {sec: 1514423498, nanosec: 147483647, type: 'ros-clock'},
    //   expected: {sec: 2, nanosec: 147483647},
    // },
    // {
    //   start:    {sec: 1514423496, nanosec: 0, type: 'ros-clock'},
    //   finish:   {sec: 1514423498, nanosec: 147483648, type: 'ros-clock'},
    //   expected: {sec: 2, nanosec: 147483648},
    // },
    // {
    //   start:    {sec: 0, nanosec: 0, type: 'ros-clock'},
    //   finish:   {sec: 100, nanosec: 0, type: 'ros-clock'},
    //   expected: {sec: 100, nanosec: 0},
    // },
    // {
    //   start:    {sec: 0, nanosec: 0, type: 'ros-clock'},
    //   finish:   {sec: 2147483647, nanosec: 0, type: 'ros-clock'},
    //   expected: {sec: 2147483647, nanosec: 0},
    // },
  ].forEach((testData, index) => {
    it('Verify Time.diff(), case#' + index, function() {
      const c = new rclnodejs.Time('system-clock');
      const delta = rclnodejs.Time.diff(testData.start, testData.finish);
      assert(delta.sec == testData.expected.sec);
      assert(delta.nanosec == testData.expected.nanosec);
    });
  });

});
