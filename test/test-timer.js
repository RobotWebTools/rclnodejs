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
const DistroUtils = require('../lib/distro.js');

const TIMER_INTERVAL = BigInt('100000000');
describe('rclnodejs Timer class testing', function () {
  this.timeout(60 * 1000);

  before(function () {
    return rclnodejs.init();
  });

  after(function () {
    rclnodejs.shutdown();
  });

  describe('Factory method createNode from node', function () {
    it('node.createTimer', function (done) {
      var times = 0;
      var node = rclnodejs.createNode('timer');
      var timer = node.createTimer(TIMER_INTERVAL, function () {
        times += 1;
        timer.cancel();
        node.destroy();
        assert.deepEqual(times, 1);
        done();
      });
      rclnodejs.spin(node);
    });
  });

  describe('Timer class methods', function () {
    var node;
    beforeEach(function () {
      node = rclnodejs.createNode('timer');
    });

    afterEach(function () {
      node.destroy();
    });

    it('timer.period should be readonly', function (done) {
      var timer = node.createTimer(TIMER_INTERVAL, function () {
        assert.deepEqual(timer.period, TIMER_INTERVAL);
        assert.throws(
          () => {
            timer.period = TIMER_INTERVAL * 2n;
          },
          function (err) {
            if (
              err instanceof TypeError &&
              /Cannot set property period/.test(err)
            ) {
              return true;
            }
            return false;
          },
          'timer.period is readonly!'
        );
        timer.cancel();
        done();
      });
      rclnodejs.spin(node);
    });

    it('timer.cancel', function (done) {
      var timer = node.createTimer(TIMER_INTERVAL, function () {
        timer.cancel();
        assert.ok(timer.isCanceled());
        done();
      });
      rclnodejs.spin(node);
    });

    it('timer.isCanceled', function (done) {
      var timer = node.createTimer(TIMER_INTERVAL, function () {
        assert.ok(!timer.isCanceled());
        timer.cancel();
        assert.ok(timer.isCanceled());
        done();
      });
      rclnodejs.spin(node);
    });

    it('timer.isReady', function (done) {
      var timer = node.createTimer(TIMER_INTERVAL, function () {
        assert.ok(!timer.isReady());
        timer.cancel();
        done();
      });
      rclnodejs.spin(node);
    });

    it('timer.reset', function (done) {
      var timer = node.createTimer(TIMER_INTERVAL, function () {
        timer.cancel();
        done();
      });
      timer.cancel();
      assert.ok(timer.isCanceled());
      timer.reset();
      assert.ok(!timer.isCanceled());
      rclnodejs.spin(node);
    });

    it('timer.timeSinceLastCall', function (done) {
      var timer = node.createTimer(TIMER_INTERVAL, function () {
        assert.deepStrictEqual(typeof timer.timeSinceLastCall(), 'bigint');
        timer.cancel();
        done();
      });
      rclnodejs.spin(node);
    });

    it('timer.timeUntilNextCall', function (done) {
      var timer = node.createTimer(TIMER_INTERVAL, function () {
        var nextCallInterval = timer.timeUntilNextCall();
        assert.deepStrictEqual(typeof nextCallInterval, 'bigint');
        assert.ok(nextCallInterval <= TIMER_INTERVAL);
        timer.cancel();
        done();
      });
      rclnodejs.spin(node);
    });

    it('timer.timerPeriod', function (done) {
      const timer = node.createTimer(BigInt('100000000'), () => {});
      assert.deepStrictEqual(timer.timerPeriod, BigInt('100000000'));
      timer.cancel();
      done();
    });

    it('timer.changeTimerPeriod', function (done) {
      const timer = node.createTimer(BigInt('100000000'), () => {});
      timer.changeTimerPeriod(BigInt('200000000'));
      assert.deepStrictEqual(timer.timerPeriod, BigInt('200000000'));
      timer.cancel();
      done();
    });

    it('timer.callTimerWithInfo', function (done) {
      if (DistroUtils.getDistroId() <= DistroUtils.getDistroId('humble')) {
        this.skip();
        return;
      }
      const timer = node.createTimer(BigInt('100000000'), () => {});
      const info = timer.callTimerWithInfo();
      assert.deepStrictEqual(typeof info.expectedCallTime, 'bigint');
      assert.deepStrictEqual(typeof info.actualCallTime, 'bigint');
      timer.cancel();
      done();
    });

    it('timer.setOnResetCallback', function (done) {
      var timer = node.createTimer(TIMER_INTERVAL, function () {});
      var called = false;
      timer.setOnResetCallback(function (events) {
        called = true;
      });
      timer.reset();

      setTimeout(() => {
        assert.ok(called);
        timer.cancel();
        done();
      }, 100);

      rclnodejs.spin(node);
    });

    it('timer.clearOnResetCallback', function (done) {
      var timer = node.createTimer(TIMER_INTERVAL, function () {});
      var called = false;
      timer.setOnResetCallback(function (events) {
        called = true;
      });
      timer.clearOnResetCallback();
      timer.reset();

      setTimeout(() => {
        assert.ok(!called);
        timer.cancel();
        done();
      }, 100);

      rclnodejs.spin(node);
    });

    it('timer callback should be called repeatedly', function (done) {
      let count = 0;
      const timer = node.createTimer(TIMER_INTERVAL, () => {
        count++;
        if (count >= 3) {
          timer.cancel();
          done();
        }
      });
      rclnodejs.spin(node);
    });
  });
});
