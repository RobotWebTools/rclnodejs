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

const TIMER_INTERVAL = Math.pow(10, 2);
describe('rclnodejs Timer class testing', function() {
  this.timeout(60 * 1000);

  before(function() {
    return rclnodejs.init();
  });

  after(function() {
    rclnodejs.shutdown();
  });

  describe('Factory method createNode from node', function() {
    it('node.createTimer', function(done) {
      var times = 0;
      var node = rclnodejs.createNode('timer');
      var timer = node.createTimer(TIMER_INTERVAL, function() {
        times += 1;
        timer.cancel();
        node.destroy();
        assert.deepEqual(times, 1);
        done();
      });
      rclnodejs.spin(node);
    });
  });

  describe('Timer class methods', function() {
    var node;
    beforeEach(function() {
      node = rclnodejs.createNode('timer');
    });

    afterEach(function() {
      node.destroy();
    });

    it('timer.period should be readonly', function(done) {
      var timer = node.createTimer(TIMER_INTERVAL, function() {
        assert.deepEqual(timer.period, TIMER_INTERVAL);
        assert.throws(
          () => {
            timer.period = TIMER_INTERVAL * 2;
          },
          function(err) {
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

    it('timer.cancel', function(done) {
      var timer = node.createTimer(TIMER_INTERVAL, function() {
        timer.cancel();
        assert.ok(timer.isCanceled());
        done();
      });
      rclnodejs.spin(node);
    });

    it('timer.isCanceled', function(done) {
      var timer = node.createTimer(TIMER_INTERVAL, function() {
        assert.ok(!timer.isCanceled());
        timer.cancel();
        assert.ok(timer.isCanceled());
        done();
      });
      rclnodejs.spin(node);
    });

    it('timer.isReady', function(done) {
      var timer = node.createTimer(TIMER_INTERVAL, function() {
        assert.ok(!timer.isReady());
        timer.cancel();
        done();
      });
      rclnodejs.spin(node);
    });

    it('timer.reset', function(done) {
      var timer = node.createTimer(TIMER_INTERVAL, function() {
        timer.cancel();
        done();
      });
      timer.cancel();
      assert.ok(timer.isCanceled());
      timer.reset();
      assert.ok(!timer.isCanceled());
      rclnodejs.spin(node);
    });

    it('timer.timeSinceLastCall', function(done) {
      var timer = node.createTimer(TIMER_INTERVAL, function() {
        assert.deepStrictEqual(typeof timer.timeSinceLastCall(), 'number');
        timer.cancel();
        done();
      });
      rclnodejs.spin(node);
    });

    it('timer.timeUntilNextCall', function(done) {
      var timer = node.createTimer(TIMER_INTERVAL, function() {
        var nextCallInterval = timer.timeUntilNextCall();
        assert.deepStrictEqual(typeof nextCallInterval, 'number');
        assert.ok(nextCallInterval <= TIMER_INTERVAL);
        timer.cancel();
        done();
      });
      rclnodejs.spin(node);
    });
  });
});
