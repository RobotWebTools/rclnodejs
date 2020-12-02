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

const IsClose = require('is-close');
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

      assertThrowsError(
        () => {
          var node = rclnodejs.createNode(nodeName, nodeNamespace);
        },
        Error,
        'must not contain characters other than',
        'Invalid node name!'
      );
    });

    it('Try creating a node with an invliad relative nampespace', function() {
      var nodeName = 'example_node_with_invalid_rel_ns',
        nodeNamespace = 'ns?';

      assertThrowsError(
        () => {
          var node = rclnodejs.createNode(nodeName, nodeNamespace);
        },
        Error,
        'must not contain characters other than',
        'Invalid relative namespace name!'
      );
    });

    it('Try creating a node with an invalid absolute namespace', function() {
      var nodeName = 'example_node_with_abs_ns',
        nodeNamespace = '/ns?';

      assertThrowsError(
        () => {
          var node = rclnodejs.createNode(nodeName, nodeNamespace);
        },
        Error,
        'must not contain characters other than',
        'Invalid absolute namespace name!'
      );
    });

    it('Try creating a node with invalid type of parameters', function() {
      var invalidParams = [
        [1, '/ns'],
        [undefined, '/ns'],
        [null, '/ns'],
        [false, '/ns'],
        [{ name: 'invalidName' }, '/ns'],
        ['validName', 2],
        ['validName', true],
        ['validName', { ns: '/invalidns' }],
        ['validName', ['ns', 3]],
        ['validName', new RegExp('abc')],
        [undefined, null],
      ];

      invalidParams.forEach(function(param) {
        assertThrowsError(
          () => {
            var node = rclnodejs.createNode(param[0], param[1]);
          },
          TypeError,
          'Invalid argument',
          'The parameters type is invalid!'
        );
      });
    });
  });
});

describe('rcl node methods testing', function() {
  var node;
  var RclString, GetParameters;
  this.timeout(60 * 1000);

  before(function() {
    return rclnodejs.init();
  });

  after(function() {
    rclnodejs.shutdown();
  });

  beforeEach(function() {
    node = rclnodejs.createNode('my_node', '/my_ns');
    RclString = 'std_msgs/msg/String';
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
    node.createPublisher(RclString, 'chatter');

    var invalidParams = [
      ['chatter?', Error, /topic name is invalid/],
      ['/chatter/42_is_the_answer', Error, /topic name is invalid/],
      ['/chatter/{bad_sub}', Error, /unknown substitution/],
    ];

    invalidParams.forEach(function(invalidParam) {
      assertThrowsError(
        () => {
          node.createPublisher(RclString, invalidParam[0]);
        },
        invalidParam[1],
        invalidParam[2],
        'Failed to createPublisher'
      );
    });
  });

  it('node.createPublisher with invalid type parameters', function() {
    var errorRegExp = new RegExp('Invalid argument');
    var invalidParams = [
      [1, 'validServiceName'],
      [undefined, 'validServiceName'],
      [null, 'validServiceName'],
      [true, 'validServiceName'],
      [{ f: 'abc' }, 'validServiceName'],
      [['a', 'b', 'c'], 'validSerivceName'],
      [RclString, 2],
      [RclString, undefined],
      [RclString, null],
      [RclString, false],
      [RclString, { n: 'invalidServiceName' }],
      [RclString, [1, 2, 3]],
      [undefined, null],
    ];

    invalidParams.forEach(function(invalidParam) {
      assert.throws(() => {
        node.createPublisher(invalidParam[0], invalidParam[1]);
      });
    });
  });

  it('node.createSubscription', function() {
    node.createSubscription(RclString, 'chatter', () => {});

    var invalidParams = [
      ['chatter?', /topic name is invalid/],
      ['/chatter/42_is_the_answer', /topic name is invalid/],
      ['/chatter/{bad_sub}', /unknown substitution/],
    ];

    invalidParams.forEach(function(invalidParam) {
      assertThrowsError(
        () => {
          node.createSubscription(RclString, invalidParam[0], () => {});
        },
        Error,
        invalidParam[1],
        'Failed to createSubscription'
      );
    });
  });

  it('node.createSubscription with invalid type parameters', function() {
    var errorRegExp = new RegExp('Invalid argument');
    var invalidParams = [
      [1, 'validTopicName', null],
      [undefined, 'validTopicName', undefined],
      [null, 'validTopicName', null],
      [true, 'validTopicName', undefined],
      [{ f: 'abc' }, 'validTopicName', null],
      [['a', 'b', 'c'], 'validTopicName', undefined],
      [RclString, 2, null],
      [RclString, undefined, undefined],
      [RclString, null, null],
      [RclString, false, undefined],
      [RclString, { n: 'invalidServiceName' }, null],
      [RclString, [1, 2, 3], undefined],
      [undefined, null, null],
    ];

    invalidParams.forEach(function(invalidParam) {
      assert.throws(() => {
        node.createSubscription(
          invalidParam[0],
          invalidParam[1],
          invalidParam[2]
        );
      });
    });
  });

  it('node.createClient', function() {
    node.createClient(GetParameters, 'get/parameters');

    var invalidParams = [
      ['get/parameters?', /topic name is invalid/],
      ['get/42parameters', /topic name is invalid/],
      ['foo/{bad_sub}', /unknown substitution/],
    ];

    invalidParams.forEach(function(invalidParam) {
      assertThrowsError(
        () => {
          node.createClient(GetParameters, invalidParam[0]);
        },
        Error,
        invalidParam[1],
        'Failed to createClient'
      );
    });
  });

  it('node.createClient with invalid type parameters', function() {
    var errorRegExp = new RegExp('Invalid argument');
    var invalidParams = [
      [1, 'validServiceName'],
      [undefined, 'validServiceName'],
      [null, 'validServiceName'],
      [true, 'validServiceName'],
      [{ f: 'abc' }, 'validServiceName'],
      [['a', 'b', 'c'], 'validServiceName'],
      [GetParameters, 2],
      [GetParameters, undefined],
      [GetParameters, null],
      [GetParameters, false],
      [GetParameters, { n: 'invalidServiceName' }],
      [GetParameters, [1, 2, 3]],
      [undefined, null],
    ];

    invalidParams.forEach(function(invalidParam) {
      assert.throws(() => {
        node.createClient(invalidParam[0], invalidParam[1]);
      });
    });
  });

  it('node.createService', function() {
    node.createService(GetParameters, 'get/parameters', () => {});

    var invalidParams = [
      ['get/parameters?', /topic name is invalid/],
      ['get/42parameters', /topic name is invalid/],
      ['foo/{bad_sub}', /unknown substitution/],
    ];

    invalidParams.forEach(function(invalidParam) {
      assertThrowsError(
        () => {
          node.createService(GetParameters, invalidParam[0], () => {});
        },
        Error,
        invalidParam[1],
        'Failed to createService'
      );
    });
  });

  it('node.createService with invalid type parameters', function() {
    var errorRegExp = new RegExp('Invalid argument');
    var invalidParams = [
      [1, 'validTopicName', null],
      [undefined, 'validTopicName', undefined],
      [null, 'validTopicName', null],
      [true, 'validTopicName', undefined],
      [{ f: 'abc' }, 'validTopicName', null],
      [['a', 'b', 'c'], 'validTopicName', undefined],
      [GetParameters, 2, null],
      [GetParameters, undefined, undefined],
      [GetParameters, null, null],
      [GetParameters, false, undefined],
      [GetParameters, { n: 'invalidServiceName' }, null],
      [GetParameters, [1, 2, 3], undefined],
      [undefined, null, null],
    ];

    invalidParams.forEach(function(invalidParam) {
      assert.throws(() => {
        node.createService(invalidParam[0], invalidParam[1], invalidParam[2]);
      });
    });
  });

  it('node.getLogger', function() {
    var logger = node.getLogger();
    assert.ok(logger);
    assert.equal(logger.debug('message debug'), false);
    assert.equal(logger.info('message info'), true);
    assert.equal(logger.warn('message warn'), true);
    assert.equal(logger.error('message error'), true);
    assert.equal(logger.fatal('message fatal'), true);
  });

  it('node.getClock', function() {
    var clock = node.getClock();
    assert.ok(clock);
    assert.strictEqual(clock.clockType, rclnodejs.ClockType.ROS_TIME);
  });

  it('node.now', function() {
    const time = node.now();
    assert.ok(time);
    assert.strictEqual(node.getClock().clockType, time.clockType);

    const seconds = time.secondsAndNanoseconds.seconds;
    const dateSeconds = Date.now() / 1000;
    assert.ok(IsClose.isClose(seconds, dateSeconds, 1));
  });

  it('node.getNodeNames', function() {
    var nodeNames = node.getNodeNames();

    var currentNode = nodeNames.indexOf('my_node');

    assert.notStrictEqual(currentNode, -1);
  });

  it('node.getNodeNamesAndNamespaces', function() {
    var nodeNames = node.getNodeNamesAndNamespaces();

    var currentNode = nodeNames.find(function(nodeName) {
      return nodeName.name === 'my_node';
    });

    assert.ok(currentNode);
    assert.strictEqual(currentNode.namespace, '/my_ns');
  });

  it('node.countPublishers', function() {
    assert.strictEqual(node.countPublishers('chatter'), 0);

    node.createPublisher(RclString, 'chatter');
    assert.strictEqual(node.countPublishers('chatter'), 1);

    node.createPublisher(RclString, 'chatter');
    assert.strictEqual(node.countPublishers('chatter'), 2);
  });

  it('node.countSubscribers', function() {
    assert.strictEqual(node.countSubscribers('chatter'), 0);

    node.createSubscription(RclString, 'chatter', () => {});
    assert.strictEqual(node.countSubscribers('chatter'), 1);

    node.createSubscription(RclString, 'chatter', () => {});
    assert.strictEqual(node.countSubscribers('chatter'), 2);
  });
});

describe('topic & serviceName getter/setter', function() {
  const RclString = 'std_msgs/msg/String';
  const AddTwoInts = 'example_interfaces/srv/AddTwoInts';

  this.timeout(60 * 1000);

  before(function() {
    return rclnodejs.init();
  });

  after(function() {
    rclnodejs.shutdown();
  });

  it('publisher: topic property getter', function() {
    var node = rclnodejs.createNode('publisher', '/topic_getter');
    var publisher = node.createPublisher(RclString, 'chatter');
    assert.deepStrictEqual(publisher.topic, 'chatter');
    node.destroy();
  });

  it('subscription: topic property getter', function() {
    var node = rclnodejs.createNode('subscription', '/topic_getter');
    var subscription = node.createSubscription(RclString, 'chatter', msg => {});
    assert.deepStrictEqual(subscription.topic, 'chatter');
    node.destroy();
  });

  it('client: serviceName property getter', function() {
    var node = rclnodejs.createNode('client', '/servicename_getter');
    var client = node.createClient(AddTwoInts, 'add_two_ints');
    assert.deepStrictEqual(client.serviceName, 'add_two_ints');
    node.destroy();
  });

  it('service: topic property getter', function() {
    var node = rclnodejs.createNode('service', '/servicename_getter');
    var service = node.createService(AddTwoInts, 'add_two_ints', req => {});
    assert.deepStrictEqual(service.serviceName, 'add_two_ints');
    node.destroy();
  });
});
