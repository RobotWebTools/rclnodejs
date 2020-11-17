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

describe('rclnodejs init and shutdown test suite', function() {

  it('basic rclnodejs.init() & rclnodejs.shutdown()', async function () {
    await rclnodejs.init();
    rclnodejs.shutdown();
  });

  it('rclnodejs.isShutdown() values', async function() {
    assert.deepStrictEqual(rclnodejs.isShutdown(), true);
    await rclnodejs.init();
    assert.deepStrictEqual(rclnodejs.isShutdown(), false);
    rclnodejs.shutdown();
    assert.deepStrictEqual(rclnodejs.isShutdown(), true);
  });

  it('rclnodejs init shutdown sequence', async function() {
    // the first round
    await rclnodejs.init();
    rclnodejs.shutdown();

    // and the second round: do it again!
    assert.doesNotThrow(async () => {
      await rclnodejs.init();
    }, "re-initializing after a shutdown should work");
    rclnodejs.shutdown();
  });

  it('rclnodejs.init(argv)', async function() {
    await rclnodejs.init(rclnodejs.Context.defaultContext(), ['a', 'b'])
    rclnodejs.shutdown();
  });

  it('rclnodejs.init(argv) - invalid argv', async function() {
    assert.throws(async () => {
      await rclnodejs.init(rclnodejs.Context.defaultContext(), 'foobar')
    }, Error, "an invalid argument should be rejected")
    rclnodejs.shutdown();
  });

  it('rclnodejs.init(argv) with null argv elements', async function() {
    assert.throws(async () => {
      await rclnodejs.init(rclnodejs.Context.defaultContext(), ['a', null, 'b'])
    }, Error, "an invalid argument should be rejected")
    rclnodejs.shutdown();
  });

  it('rclnodejs double init should throw', async function() {
    await rclnodejs.init();
    assert.throws(async () => {
      await rclnodejs.init();
    }, Error, "initializing it twice shall cause an error to be thrown")
  });

  it('rclnodejs double shutdown should work', async function() {
    await rclnodejs.init();
    rclnodejs.shutdown();

    assert.doesNotThrow(() => {
      rclnodejs.shutdown();
    }, "shutting rclnodejs down twice should not cause an error to be thrown")
  });

  it('rclnodejs create node without init should fail', async function() {
    assert.throws(() => {
      rclnodejs.createNode('my_node');
    }, Error, "creating a node on an uninitialized context should cause an error to be thrown");
  });

  it('rclnodejs multiple contexts init shutdown sequence', async function() {
    async function initShutdownSequence() {
      // init the default context
      assert.ok(rclnodejs.isShutdown());
      await rclnodejs.init();
      assert.ok(!rclnodejs.isShutdown());
      const defaultCtx = rclnodejs.Context.defaultContext();
      assert.ok(defaultCtx !== null);
      assert.ok(defaultCtx.isOk);
      assert.ok(defaultCtx.isDefaultContext);

      // init another one
      const ctx = new rclnodejs.Context();
      await rclnodejs.init(ctx);
      assert.ok(!rclnodejs.isShutdown(ctx));
      assert.ok(!rclnodejs.isShutdown());
      assert.ok(ctx.isOk);
      assert.ok(!ctx.isDefaultContext);
      assert.ok(defaultCtx.isOk);
      assert.ok(defaultCtx.isDefaultContext);

      // shut down the default context
      rclnodejs.shutdown();
      assert.ok(rclnodejs.isShutdown());
      assert.ok(!rclnodejs.isShutdown(ctx));
      assert.ok(ctx.isOk);
      assert.ok(!defaultCtx.isOk);

      // shut down the other context
      rclnodejs.shutdown(ctx);
      assert.ok(rclnodejs.isShutdown());
      assert.ok(rclnodejs.isShutdown(ctx));
      assert.ok(!ctx.isOk);
      assert.ok(!defaultCtx.isOk);
    }

    // execute it twice
    await initShutdownSequence();
    await initShutdownSequence();
  });

});
