// Copyright (c) 2026, The Robot Web Tools Contributors
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

describe('spinUntilFutureComplete tests', function () {
  this.timeout(60 * 1000);

  let node;

  before(function () {
    return rclnodejs.init();
  });

  after(function () {
    rclnodejs.shutdown();
  });

  beforeEach(function () {
    node = rclnodejs.createNode('spin_future_test_node');
  });

  afterEach(function () {
    if (node.spinning) {
      node.stop();
    }
    node.destroy();
  });

  it('should resolve when promise resolves', async function () {
    const result = await node.spinUntilFutureComplete(Promise.resolve(42));
    assert.strictEqual(result, 42);
    assert.strictEqual(node.spinning, false);
  });

  it('should reject when promise rejects', async function () {
    await assert.rejects(
      () =>
        node.spinUntilFutureComplete(Promise.reject(new Error('test error'))),
      { message: 'test error' }
    );
    assert.strictEqual(node.spinning, false);
  });

  it('should timeout when promise does not resolve', async function () {
    const neverResolves = new Promise(() => {});
    await assert.rejects(
      () => node.spinUntilFutureComplete(neverResolves, 500),
      { message: /timed out/ }
    );
    assert.strictEqual(node.spinning, false);
  });

  it('should not stop spinning if node was already spinning', async function () {
    node.spin();
    assert.strictEqual(node.spinning, true);

    const result = await node.spinUntilFutureComplete(Promise.resolve('hello'));
    assert.strictEqual(result, 'hello');
    assert.strictEqual(node.spinning, true); // still spinning
  });

  it('should stop spinning after completion if it started spinning', async function () {
    assert.strictEqual(node.spinning, false);

    const result = await node.spinUntilFutureComplete(
      new Promise((resolve) => setTimeout(() => resolve('delayed'), 100))
    );
    assert.strictEqual(result, 'delayed');
    assert.strictEqual(node.spinning, false);
  });

  it('should work with delayed promises', async function () {
    const result = await node.spinUntilFutureComplete(
      new Promise((resolve) => setTimeout(() => resolve('ok'), 200)),
      5000
    );
    assert.strictEqual(result, 'ok');
  });

  it('should throw for invalid promise argument', async function () {
    await assert.rejects(() => node.spinUntilFutureComplete('not a promise'));
    await assert.rejects(() => node.spinUntilFutureComplete(null));
  });

  it('should work via module-level function', async function () {
    const result = await rclnodejs.spinUntilFutureComplete(
      node,
      Promise.resolve('module-level')
    );
    assert.strictEqual(result, 'module-level');
  });
});
