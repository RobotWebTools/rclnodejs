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
const utils = require('./utils.js');

describe('rclnodejs guard condition test suite', function() {
  var node;
  var timeout = 10;
  this.timeout(60 * 1000);

  before(function() {
    return rclnodejs.init();
  });

  after(function() {
    rclnodejs.shutdown();
  });

  beforeEach(function() {
    node = rclnodejs.createNode('guard_node');
    rclnodejs.spin(node, timeout);
  });

  afterEach(function() {
    node.destroy();
  });

  it('Test trigger', async function() {
    let called = false;

    function func() {
      called = true;
    }

    const gc = node.createGuardCondition(func);

    await utils.delay(timeout);
    assert.strictEqual(called, false);

    gc.trigger();
    await utils.delay(timeout);
    assert.strictEqual(called, true);

    node.destroyGuardCondition(gc);
  });

  it('Test double trigger', async function() {
    let called1 = false;
    let called2 = false;

    function func1() {
      called1 = true;
    }

    function func2() {
      called2 = true;
    }

    const gc1 = node.createGuardCondition(func1);
    const gc2 = node.createGuardCondition(func2);

    await utils.delay(timeout);
    assert.strictEqual(called1, false);
    assert.strictEqual(called2, false);

    gc1.trigger();
    gc2.trigger();
    await utils.delay(timeout);
    assert.strictEqual(called1, true);
    assert.strictEqual(called2, true);

    called1 = false;
    called2 = false;
    await utils.delay(timeout);
    assert.strictEqual(called1, false);
    assert.strictEqual(called2, false);

    node.destroyGuardCondition(gc1);
    node.destroyGuardCondition(gc2);
  });
});
