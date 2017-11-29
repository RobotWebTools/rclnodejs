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
const assertUtils = require('./utils.js');
const assertThrowsError = assertUtils.assertThrowsError;

describe('rclnodejs node test suite', function() {
  this.timeout(60 * 1000);

  before(function() {
    return rclnodejs.init();
  });

  after(function() {
    rclnodejs.shutdown();
  });

  describe('createNode method testing', function() {
    it('Try creating a node', function() {
      var node = rclnodejs.createNode('example_node');
      node.destroy();
    });

    it('Try creating a node with a namespace', function() {
      let nodeName = 'example_node_with_ns',
        nodeNamespace = '/ns';

      var node = rclnodejs.createNode(nodeName, nodeNamespace);
      assert.deepStrictEqual(node.namespace(), '/ns');
    });

    it('Try creating a node with the empty namespace', function() {
      let nodeName = 'example_node_with_empty_ns',
        nodeNamespace = '';

      var node = rclnodejs.createNode(nodeName, nodeNamespace);
      assert.deepStrictEqual(node.namespace(), '/');
    });

    it('Try creating a node with a relative namespace', function() {
      let nodeName = 'example_node_with_rel_ns',
        nodeNamespace = 'ns';

      var node = rclnodejs.createNode(nodeName, nodeNamespace);
      assert.deepStrictEqual(node.namespace(), '/ns');
    });

    it('Try creating a node with an invalid name', function() {
      var nodeName = 'example_node_invalid_name?',
        nodeNamespace = 'ns';

      assertThrowsError(() => {
        var node = rclnodejs.createNode(nodeName, nodeNamespace);
      }, Error, 'must not contain characters other than', 'Invalid node name!');
    });

    it('Try creating a node with an invliad relative nampespace', function() {
      var nodeName = 'example_node_with_invalid_rel_ns',
        nodeNamespace = 'ns?';

      assertThrowsError(() => {
        var node = rclnodejs.createNode(nodeName, nodeNamespace);
      }, Error, 'must not contain characters other than', 'Invalid relative namespace name!');
    });

    it('Try creating a node with an invalid absolute namespace', function() {
      var nodeName = 'example_node_with_abs_ns',
        nodeNamespace = '/ns?';

      assertThrowsError(() => {
        var node = rclnodejs.createNode(nodeName, nodeNamespace);
      }, Error, 'must not contain characters other than', 'Invalid absolute namespace name!');
    });

    it('Try creating a node with invalid type of parameters', function() {
      var invalidParams = [
        [1, '/ns'],
        [undefined, '/ns'],
        [null, '/ns'],
        [false, '/ns'],
        [{name: 'invalidName'}, '/ns'],
        ['validName', 2],
        ['validName', true],
        ['validName', {ns: '/invalidns'}],
        ['validName', ['ns', 3]],
        ['validName', new RegExp('abc')],
        [undefined, null]
      ];

      invalidParams.forEach(function(param) {
        assertThrowsError(() => {
          var node = rclnodejs.createNode(param[0], param[1]);
        }, TypeError, 'Invalid argument', 'The parameters type is invalid!');
      });
    });
  });
});

describe('rcl node methods testing', function() {
  var node;
  var rclString, GetParameters;
  this.timeout(60 * 1000);

  before(function() {
    return rclnodejs.init();
  });

  after(function() {
    rclnodejs.shutdown();
  });

  beforeEach(function() {
    node = rclnodejs.createNode('my_node', '/my_ns');
    rclString = 'std_msgs/msg/String';
    GetParameters = 'rcl_interfaces/srv/GetParameters';
  });

  afterEach(function() {
    node.destroy();
  });

  it('node setter/getter', function() {
    assert.notDeepStrictEqual(null, node);

    assert.deepStrictEqual(node.name(), 'my_node');
    assert.deepStrictEqual(node.namespace(), '/my_ns');
  });

  it('node.createPublisher', function() {
    node.createPublisher(rclString, 'chatter');

    var invalidParams = [
      ['chatter?', Error, /topic name is invalid/],
      ['/chatter/42_is_the_answer', Error, /topic name is invalid/],
      ['/chatter/{bad_sub}', Error, /unknown substitution/]
    ];

    invalidParams.forEach(function(invalidParam) {
      assertThrowsError(() => {
        node.createPublisher(rclString, invalidParam[0]);
      }, invalidParam[1], invalidParam[2], 'Failed to createPublisher');
    });
  });

  it('node.createPublisher with invalid type parameters', function() {
    var errorRegExp = new RegExp('Invalid argument');
    var invalidParams = [
      [1, 'validServiceName'],
      [undefined, 'validServiceName'],
      [null, 'validServiceName'],
      [true, 'validServiceName'],
      [{f: 'abc'}, 'validServiceName'],
      [['a', 'b', 'c'], 'validSerivceName'],
      [rclString, 2],
      [rclString, undefined],
      [rclString, null],
      [rclString, false],
      [rclString, {n: 'invalidServiceName'}],
      [rclString, [1, 2, 3]],
      [undefined, null]
    ];

    invalidParams.forEach(function(invalidParam) {
      assertThrowsError(() => {
        node.createPublisher(invalidParam[0], invalidParam[1]);
      }, TypeError, errorRegExp, 'Failed to createPublisher');
    });
  });

  it('node.createSubscription', function() {
    node.createSubscription(rclString, 'chatter', () => {});

    var invalidParams = [
      ['chatter?', /topic name is invalid/],
      ['/chatter/42_is_the_answer', /topic name is invalid/],
      ['/chatter/{bad_sub}', /unknown substitution/]
    ];

    invalidParams.forEach(function(invalidParam) {
      assertThrowsError(() => {
        node.createSubscription(rclString, invalidParam[0], () => {});
      }, Error, invalidParam[1], 'Failed to createSubscription');
    });
  });

  it('node.createSubscription with invalid type parameters', function() {
    var errorRegExp = new RegExp('Invalid argument');
    var invalidParams = [
      [1, 'validTopicName', null],
      [undefined, 'validTopicName', undefined],
      [null, 'validTopicName', null],
      [true, 'validTopicName', undefined],
      [{f: 'abc'}, 'validTopicName', null],
      [['a', 'b', 'c'], 'validTopicName', undefined],
      [rclString, 2, null],
      [rclString, undefined, undefined],
      [rclString, null, null],
      [rclString, false, undefined],
      [rclString, {n: 'invalidServiceName'}, null],
      [rclString, [1, 2, 3], undefined],
      [undefined, null, null]
    ];

    invalidParams.forEach(function(invalidParam) {
      assertThrowsError(() => {
        node.createSubscription(invalidParam[0], invalidParam[1], invalidParam[2]);
      }, TypeError, errorRegExp, 'Failed to createSubscription');
    });
  });

  it('node.createClient', function() {
    node.createClient(GetParameters, 'get/parameters');

    var invalidParams = [
      ['get/parameters?', /topic name is invalid/], ,
      ['get/42parameters', /topic name is invalid/],
      ['foo/{bad_sub}', /unknown substitution/]
    ];

    invalidParams.forEach(function(invalidParam) {
      assertThrowsError(() => {
        node.createClient(GetParameters, invalidParam[0]);
      }, Error, invalidParam[1], 'Failed to createClient');
    });
  });

  it('node.createClient with invalid type parameters', function() {
    var errorRegExp = new RegExp('Invalid argument');
    var invalidParams = [
      [1, 'validServiceName'],
      [undefined, 'validServiceName'],
      [null, 'validServiceName'],
      [true, 'validServiceName'],
      [{f: 'abc'}, 'validServiceName'],
      [['a', 'b', 'c'], 'validServiceName'],
      [GetParameters, 2],
      [GetParameters, undefined],
      [GetParameters, null],
      [GetParameters, false],
      [GetParameters, {n: 'invalidServiceName'}],
      [GetParameters, [1, 2, 3]],
      [undefined, null]
    ];

    invalidParams.forEach(function(invalidParam) {
      assertThrowsError(() => {
        node.createClient(invalidParam[0], invalidParam[1]);
      }, TypeError, errorRegExp, 'Failed to createClient');
    });
  });

  it('node.createService', function() {
    node.createService(GetParameters, 'get/parameters', () => {});

    var invalidParams = [
      ['get/parameters?', /topic name is invalid/],
      ['get/42parameters', /topic name is invalid/],
      ['foo/{bad_sub}', /unknown substitution/]
    ];

    invalidParams.forEach(function(invalidParam) {
      assertThrowsError(() => {
        node.createService(GetParameters, invalidParam[0], () => {});
      }, Error, invalidParam[1], 'Failed to createService');
    });
  });

  it('node.createService with invalid type parameters', function() {
    var errorRegExp = new RegExp('Invalid argument');
    var invalidParams = [
      [1, 'validTopicName', null],
      [undefined, 'validTopicName', undefined],
      [null, 'validTopicName', null],
      [true, 'validTopicName', undefined],
      [{f: 'abc'}, 'validTopicName', null],
      [['a', 'b', 'c'], 'validTopicName', undefined],
      [GetParameters, 2, null],
      [GetParameters, undefined, undefined],
      [GetParameters, null, null],
      [GetParameters, false, undefined],
      [GetParameters, {n: 'invalidServiceName'}, null],
      [GetParameters, [1, 2, 3], undefined],
      [undefined, null, null]
    ];

    invalidParams.forEach(function(invalidParam) {
      assertThrowsError(() => {
        node.createService(invalidParam[0], invalidParam[1], invalidParam[2]);
      }, TypeError, errorRegExp, 'Failed to createService');
    });
  });
});
