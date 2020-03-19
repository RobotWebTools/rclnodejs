// Copyright (c) 2020 Matt Richard. All rights reserved.
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
const assertUtils = require('./utils.js');
const rclnodejs = require('../index.js');

describe('rclnodejs action graph', function() {
  let node1;
  let node2;
  let node3;
  let fibonacci = 'rclnodejs_test_msgs/action/Fibonacci';
  this.timeout(60 * 1000);

  const NODE1_NAME = 'node1';
  const NODE1_NS = '/';
  const NODE2_NAME = 'node2';
  const NODE2_NS = '/node2_ns';
  const NODE3_NAME = 'node3';
  const NODE3_NS = '/node3_ns';

  const ACTION1_NAME = 'action1';
  const ACTION2_NAME = 'action2';

  async function waitForNodes() {
    let deadline = Date.now() + 2000;
    let nodeNames = node1.getNodeNames();
    while (
      Date.now() < deadline &&
      (nodeNames.indexOf(NODE2_NAME) === -1 ||
        nodeNames.indexOf(NODE3_NAME) === -1)
    ) {
      await assertUtils.createDelay(100);
      nodeNames = node1.getNodeNames();
    }
  }

  before(function() {
    return rclnodejs.init();
  });

  after(function() {
    rclnodejs.shutdown();
  });

  beforeEach(function() {
    node1 = rclnodejs.createNode(NODE1_NAME, NODE1_NS);
    node2 = rclnodejs.createNode(NODE2_NAME, NODE2_NS);
    node3 = rclnodejs.createNode(NODE3_NAME, NODE3_NS);

    new rclnodejs.ActionClient(node2, fibonacci, ACTION1_NAME);
    new rclnodejs.ActionServer(node2, fibonacci, ACTION1_NAME, () => null);
    new rclnodejs.ActionClient(node3, fibonacci, ACTION1_NAME);
    new rclnodejs.ActionServer(node3, fibonacci, ACTION1_NAME, () => null);
    new rclnodejs.ActionClient(node3, fibonacci, ACTION2_NAME);
    new rclnodejs.ActionServer(node3, fibonacci, ACTION2_NAME, () => null);

    // Wait for nodes to discover each other
    return waitForNodes();
  });

  afterEach(function() {
    node1.destroy();
    node2.destroy();
    node3.destroy();
  });

  it('Test getActionClientNamesAndTypesByNode', function() {
    let result = rclnodejs.getActionClientNamesAndTypesByNode(
      node2,
      NODE1_NAME,
      NODE1_NS
    );
    assert.strictEqual(result.length, 0);

    result = rclnodejs.getActionClientNamesAndTypesByNode(
      node1,
      NODE2_NAME,
      NODE2_NS
    );
    assert.strictEqual(result.length, 1);

    let name = result[0].name;
    let types = result[0].types;
    assert.strictEqual(name, `${NODE2_NS}/${ACTION1_NAME}`);
    assert.strictEqual(types.length, 1);
    assert.strictEqual(types[0], fibonacci);

    result = rclnodejs.getActionClientNamesAndTypesByNode(
      node1,
      NODE3_NAME,
      NODE3_NS
    );
    assert.strictEqual(result.length, 2);

    types = result[0].types;
    assert.strictEqual(types.length, 1);
    assert.strictEqual(types[0], fibonacci);
    types = result[1].types;
    assert.strictEqual(types.length, 1);
    assert.strictEqual(types[0], fibonacci);

    assert.notStrictEqual(
      result.findIndex(r => r.name === `${NODE3_NS}/${ACTION1_NAME}`),
      -1
    );
    assert.notStrictEqual(
      result.findIndex(r => r.name === `${NODE3_NS}/${ACTION2_NAME}`),
      -1
    );
  });

  it('Test getActionServerNamesAndTypesByNode', function() {
    let result = rclnodejs.getActionServerNamesAndTypesByNode(
      node2,
      NODE1_NAME,
      NODE1_NS
    );
    assert.strictEqual(result.length, 0);

    result = rclnodejs.getActionServerNamesAndTypesByNode(
      node1,
      NODE2_NAME,
      NODE2_NS
    );
    assert.strictEqual(result.length, 1);

    let name = result[0].name;
    let types = result[0].types;
    assert.strictEqual(name, `${NODE2_NS}/${ACTION1_NAME}`);
    assert.strictEqual(types.length, 1);
    assert.strictEqual(types[0], fibonacci);

    result = rclnodejs.getActionServerNamesAndTypesByNode(
      node1,
      NODE3_NAME,
      NODE3_NS
    );
    assert.strictEqual(result.length, 2);

    types = result[0].types;
    assert.strictEqual(types.length, 1);
    assert.strictEqual(types[0], fibonacci);
    types = result[1].types;
    assert.strictEqual(types.length, 1);
    assert.strictEqual(types[0], fibonacci);

    assert.notStrictEqual(
      result.findIndex(r => r.name === `${NODE3_NS}/${ACTION1_NAME}`),
      -1
    );
    assert.notStrictEqual(
      result.findIndex(r => r.name === `${NODE3_NS}/${ACTION2_NAME}`),
      -1
    );
  });

  it('Test getActionNamesAndTypes', function() {
    let result = rclnodejs.getActionNamesAndTypes(node1);
    assert.strictEqual(result.length, 3);

    let types = result[0].types;
    assert.strictEqual(types.length, 1);
    assert.strictEqual(types[0], fibonacci);
    types = result[1].types;
    assert.strictEqual(types.length, 1);
    assert.strictEqual(types[0], fibonacci);
    types = result[2].types;
    assert.strictEqual(types.length, 1);
    assert.strictEqual(types[0], fibonacci);

    assert.notStrictEqual(
      result.findIndex(r => r.name === `${NODE2_NS}/${ACTION1_NAME}`),
      -1
    );
    assert.notStrictEqual(
      result.findIndex(r => r.name === `${NODE3_NS}/${ACTION1_NAME}`),
      -1
    );
    assert.notStrictEqual(
      result.findIndex(r => r.name === `${NODE3_NS}/${ACTION2_NAME}`),
      -1
    );
  });
});
