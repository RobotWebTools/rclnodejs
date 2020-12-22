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

describe('context test suite', function () {
  this.timeout(60 * 1000);

  it('context constructor', function (done) {
    let context = new rclnodejs.Context();
    assert.ok(context.isUninitialized());
    assert.ok(!context.isInitialized());
    assert.ok(!context.isShutdown());
    done();
  });

  it('context defaultContext', function (done) {
    let context1 = rclnodejs.Context.defaultContext();
    assert.ok(context1.isDefaultContext());

    let context2 = rclnodejs.Context.defaultContext();
    assert.ok(context2.isDefaultContext());
    assert.strictEqual(context1, context2);

    let context3 = new rclnodejs.Context();
    assert.ok(!context3.isDefaultContext());

    context1.shutdown();
    context3.shutdown();

    let context4 = rclnodejs.Context.defaultContext();
    assert.ok(context4.isDefaultContext());

    context4.shutdown();
    done();
  });

  it('context initialized', async function () {
    let context = new rclnodejs.Context();
    await rclnodejs.init(context);
    assert.ok(!context.isUninitialized());
    assert.ok(context.isInitialized());
    assert.ok(!context.isShutdown());
  });

  it('context basic shutdown', async function () {
    let context = new rclnodejs.Context();
    await rclnodejs.init(context);
    context.shutdown();
    assert.ok(!context.isUninitialized());
    assert.ok(!context.isInitialized());
    assert.ok(context.isShutdown());
  });

  it('context nodes(), onNodeCreated() & onNodeDestroyed()', async function () {
    let context = new rclnodejs.Context();
    await rclnodejs.init(context);

    let node1 = new rclnodejs.Node('n1', undefined, context);
    assert.strictEqual(context.nodes.length, 1);
    assert.strictEqual(context.nodes[0].name(), 'n1');

    let node2 = new rclnodejs.Node('n2', undefined, context);
    assert.strictEqual(context.nodes.length, 2);

    context.shutdown();
    assert.strictEqual(context.nodes.length, 0);
  });
});
