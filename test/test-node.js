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

describe('rclnodejs node test suite', function() {
  describe('createNode method testing', function() {
    before(function() {
      this.timeout(60 * 1000);
      return rclnodejs.init();
    });

    after(function() {
      rclnodejs.shutdown();
    });

    it('Try creating a node', function() {
      const node = rclnodejs.createNode('example_node');
      rclnodejs.spin(node);
      node.destroy();
    });

/* Todo:
**     need the get_name() and get_namespace() interface.
    it('Try creating a node with a namespace', function() {
      let nodeName = 'example_node_with_ns',
          nodeNamespace = '/ns';

      const node = rclnodejs.createNode(nodeName, nodeNamespace);
      rclnodejs.spin(node);
      assert.deepStrictEqual(node.getNamespace(), '/ns');
    });

    it('Try creating a node with the empty namespace', function() {
      let nodeName = 'example_node_with_empty_ns',
          nodeNamespace = '';

      const node = rclnodejs.createNode(nodeName, nodeNamespace);
      rclnodejs.spin(node);
      assert.deepStrictEqual(node.getNamespace(), '/');
    });

    it('Try creating a node with a relative namespace', function() {
      let nodeName = 'example_node_with_rel_ns',
          nodeNamespace = 'ns';

      const node = rclnodejs.createNode(nodeName, nodeNamespace);
      rclnodejs.spin(node);
      assert.deepStrictEqual(node.getNamespace(), '/ns');
    });
*/
    it('Try creating a node with an invalid name', function() {
      var nodeName = 'example_node_invalid_name?',
          nodeNamespace = 'ns';

      assert.throws(() => {
        const node = rclnodejs.createNode(nodeName, nodeNamespace);
        rclnodejs.spin(node);
        node.destroy();
      }, /must not contain characters other than/, 'Invalid node name!');
    });

    it('Try creating a node with an invliad relative nampespace', function() {
      var nodeName = 'example_node_with_invalid_rel_ns',
          nodeNamespace = 'ns?';

      assert.throws(() => {
        const node = rclnodejs.createNode(nodeName, nodeNamespace);
        rclnodejs.spin(node);
        node.destroy();
      }, /must not contain characters other than/, 'Invalid relative namespace name!');
    });

    it('Try creating a node with an invalid absolute namespace', function() {
      var nodeName = 'example_node_with_abs_ns',
          nodeNamespace = '/ns?';

      assert.throws(() => {
        const node = rclnodejs.createNode(nodeName, nodeNamespace);
        rclnodejs.spin(node);
        node.destroy();
      }, /must not contain characters other than/, 'Invalid absolute namespace name!');
    });
  });
});

describe('rcl node methods testing', function() {
  var node;
  var rclString;

  before(function() {
    this.timeout(60 * 1000);
    return rclnodejs.init();
  });

  after(function() {
    rclnodejs.shutdown();
  });

  beforeEach(function() {
    node = rclnodejs.createNode('my_node', '/my_ns');
    rclString = rclnodejs.require('std_msgs').msg.String;
  });

  afterEach(function() {
    node.destroy();
  });

  it('node setter/getter', function() {
    assert.notDeepStrictEqual(null, node);

    // Todo
    // assert.deepStrictEqual(node.getName(), 'my_node');
    // assert.deepStrictEqual(node.getNamespace(), '/my_ns');
  });

  it('node.createPublisher', function() {
    node.createPublisher(rclString, 'chatter');

    var invalidParams = [
      ['chatter?', /topic name is invalid/],
      ['/chatter/42_is_the_answer', /topic name is invalid/],
      ['/chatter/{bad_sub}', /unknown substitution/]
    ];

    invalidParams.forEach(function(invalidParam) {
      assert.throws(function() {
        node.createPublisher(rclString, invalidParam[0]);
      }, invalidParam[1], 'Failed to createPublisher');
    });
  });

  it('node.createSubscription', function() {
    node.createSubscription(rclString, 'chatter', () => {
      assert.ok(true);
    });

    var invalidParams = [
      ['chatter?', /topic name is invalid/],
      ['/chatter/42_is_the_answer', /topic name is invalid/],
      ['/chatter/{bad_sub}', /unknown substitution/]
    ];

    invalidParams.forEach(function(invalidParam) {
      assert.throws(function() {
        node.createPublisher(rclString, invalidParam[0], () => {});
      }, invalidParam[1], 'Failed to createSubscription');
    });
  });

  it('node.createClient', function() {
    var GetParameters = rclnodejs.require('rcl_interfaces').srv.GetParameters;
    node.createClient(GetParameters, 'get/parameters');

    var invalidParams = [
      ['get/parameters?', /topic name is invalid/],
      ['get/42parameters', /topic name is invalid/],
      ['foo/{bad_sub}', /unknown substitution/]
    ];

    invalidParams.forEach(function(invalidParam) {
      assert.throws(function() {
        node.createClient(GetParameters, invalidParam[0]);
      }, invalidParam[1], 'Failed to createClient');
    });
  });

  it('node.createService', function() {
    var GetParameters = rclnodejs.require('rcl_interfaces').srv.GetParameters;
    node.createService(GetParameters, 'get/parameters', () => {
      assert.ok(true);
    });

    var invalidParams = [
      ['get/parameters?', /topic name is invalid/],
      ['get/42parameters', /topic name is invalid/],
      ['foo/{bad_sub}', /unknown substitution/]
    ];

    invalidParams.forEach(function(invalidParam) {
      assert.throws(function() {
        node.createService(GetParameters, invalidParam[0], () => {});
      }, invalidParam[1], 'Failed to createService');
    });
  });
});
