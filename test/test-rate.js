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

const isClose = require('is-close');
const rclnodejs = require('../index.js');
const assertUtils = require('./utils.js');
const assertThrowsError = assertUtils.assertThrowsError;

const { performance } = require('perf_hooks');

describe('rclnodejs rate test suite', function() {
  let node;
  this.timeout(60 * 1000);

  before(function() {
    return rclnodejs.init();
  });

  after(function() {
    rclnodejs.shutdown();
  });

  beforeEach(function() {
    node = rclnodejs.createNode('rate_node');
  });

  afterEach(function() {
    node.destroy();
  });

  it('rate constructor tests', function() {
    const rate1 = node.createRate();
    assert.equal(rate1.frequency, 1);

    const rate2 = node.createRate(1000);
    assert.equal(rate2.frequency, 1000);

    const rate3 = node.createRate(0.001);
    assert.equal(rate3.frequency, 0.001);

    assertThrowsError(() => {
      node.createRate(1001);
    }, RangeError);

    assertThrowsError(() => {
      node.createRate(0);
    }, RangeError);
  });

  it('rate api tests', function() {
    const rate = node.createRate();
    assert.equal(rate.frequency, 1);
    assert.equal(rate.isCanceled(), false);
  });

  it('rate sleep cancel tests', async function() {
    const rate = node.createRate();
    assert.equal(rate.isCanceled(), false);

    rate.cancel();
    assert.ok(rate.isCanceled());

    rate.cancel(); // redundant cancel is a nop
  });

  it('rate server node naming test', async function() {
    const rate = node.createRate();
    const rateTimerServerName = `_${node.name()}_rate_timer_server`;
    assert.ok(node.getNodeNames().includes(rateTimerServerName));
  });

  it('rate sleep accuracy test, 1000 hz for 3 seconds', async function() {
    // run 3 * hz,
    //   collect and average the sleep intervals
    //   compare average sleep interval with the period of the timer
    const hz = 1000;
    const rate = node.createRate(hz);

    const dataSize = hz * 3;
    const arr = new Array(dataSize).fill(0);
    for (let i = 0; i < dataSize; i++) {
      const start = performance.now();
      await rate.sleep();
      rclnodejs.spinOnce(node, 0);
      const end = performance.now();
      arr[i] = end - start;
    }

    const avg =
      arr.reduce((prev, cur) => {
        return prev + cur;
      }, 0) / dataSize;
    // console.log('data avg: ', avg);
  });
});
