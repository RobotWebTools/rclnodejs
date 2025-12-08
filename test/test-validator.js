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

describe('rclnodejs validator testing', function () {
  this.timeout(60 * 1000);

  before(function () {
    return rclnodejs.init();
  });

  after(function () {
    rclnodejs.shutdown();
  });

  it('validate_full_topic_name', function () {
    var names = ['/chatter', '/node_name/chatter', '/ns/node_name/chatter'];
    names.forEach((name) => {
      rclnodejs.validator.validateFullTopicName(name);
    });
  });

  it('validate_full_topic_name_failure_invalid_char', function () {
    utils.assertThrowsError(
      () => {
        rclnodejs.validator.validateFullTopicName('/invalid_topic?');
      },
      Error,
      'must not contain characters',
      'invalid full topic name!'
    );
  });

  it('validate_full_name_failure_not_absolute', function () {
    utils.assertThrowsError(
      () => {
        rclnodejs.validator.validateFullTopicName('invalid_topic');
      },
      Error,
      'must be absolute',
      'invalid full topic name!'
    );
  });

  it('validate_node_name', function () {
    rclnodejs.validator.validateNodeName('my_node');
  });

  it('validate_node_name_failures', function () {
    utils.assertThrowsError(
      () => {
        rclnodejs.validator.validateNodeName('');
      },
      Error,
      'must not be empty',
      'invalid node name!'
    );

    utils.assertThrowsError(
      () => {
        rclnodejs.validator.validateNodeName('invalid_node?');
      },
      Error,
      'must not contain characters',
      'invalid node name!'
    );

    utils.assertThrowsError(
      () => {
        rclnodejs.validator.validateNodeName('/invalid_node');
      },
      Error,
      'must not contain characters',
      'invalid node name!'
    );
  });

  it('topic_or_service_is_hidden', function () {
    var tests = [
      ['/chatter', false],
      ['chatter', false],
      ['/_chatter', true],
      ['_chatter', true],
      ['/more/complex/chatter', false],
      ['/_more/complex/chatter', true],
      ['/more/_complex/chatter', true],
      ['/more/complex/_chatter', true],
      ['/more/complex_/chatter', false],
      ['/more/complex/_/chatter', true],
      ['_/chatter', true],
      ['/chatter_', false],
      ['/more_/complex/chatter', false],
      ['/more/complex_/chatter', false],
      ['/more/complex/chatter_', false],
      ['/_more/_complex/_chatter', true],
      ['', false],
      ['_', true],
    ];

    tests.forEach((test) => {
      assert.deepStrictEqual(
        rclnodejs.isTopicOrServiceHidden(test[0]),
        test[1]
      );
    });
  });

  it('validate_topic_name', function () {
    var names = ['chatter', '{node}/chatter', '~/chatter'];

    names.forEach((name) => {
      rclnodejs.validator.validateTopicName(name);
    });
  });

  it('valid_topic_name_failure_invalid_char', function () {
    utils.assertThrowsError(
      () => {
        rclnodejs.validator.validateTopicName('/invalid_topic?');
      },
      Error,
      'must not contain characters',
      'invalid topic name!'
    );
  });

  it('validate_topic_name_failure_start', function () {
    utils.assertThrowsError(
      () => {
        rclnodejs.validator.validateTopicName('invalid/42topic');
      },
      Error,
      'must not start with a number',
      'invalid topic name!'
    );
  });

  it('validate_namespace', function () {
    var names = ['/my_ns', '/'];

    names.forEach((name) => {
      rclnodejs.validator.validateNamespace(name);
    });
  });

  it('validate_namespace_failures', function () {
    utils.assertThrowsError(
      () => {
        rclnodejs.validator.validateNamespace('');
      },
      Error,
      'must not be empty',
      'invalid namespace!'
    );

    utils.assertThrowsError(
      () => {
        rclnodejs.validator.validateNamespace('invalid_namespace');
      },
      Error,
      'must be absolute',
      'invalid namespace!'
    );
  });

  describe('isValidFullTopicName', function () {
    it('should return true for valid fully-qualified topics', function () {
      assert.strictEqual(
        rclnodejs.validator.isValidFullTopicName('/chatter'),
        true
      );
      assert.strictEqual(
        rclnodejs.validator.isValidFullTopicName('/node_name/chatter'),
        true
      );
      assert.strictEqual(
        rclnodejs.validator.isValidFullTopicName('/ns/node_name/chatter'),
        true
      );
    });

    it('should return false for invalid topics', function () {
      assert.strictEqual(
        rclnodejs.validator.isValidFullTopicName('/invalid_topic?'),
        false
      );
      assert.strictEqual(
        rclnodejs.validator.isValidFullTopicName('relative_topic'),
        false
      );
    });

    it('should return false for non-string input', function () {
      assert.strictEqual(rclnodejs.validator.isValidFullTopicName(123), false);
      assert.strictEqual(rclnodejs.validator.isValidFullTopicName(null), false);
      assert.strictEqual(
        rclnodejs.validator.isValidFullTopicName(undefined),
        false
      );
    });
  });

  describe('isValidNodeName', function () {
    it('should return true for valid node names', function () {
      assert.strictEqual(rclnodejs.validator.isValidNodeName('my_node'), true);
      assert.strictEqual(rclnodejs.validator.isValidNodeName('node123'), true);
    });

    it('should return false for invalid node names', function () {
      assert.strictEqual(rclnodejs.validator.isValidNodeName(''), false);
      assert.strictEqual(
        rclnodejs.validator.isValidNodeName('invalid_node?'),
        false
      );
      assert.strictEqual(
        rclnodejs.validator.isValidNodeName('/invalid_node'),
        false
      );
    });

    it('should return false for non-string input', function () {
      assert.strictEqual(rclnodejs.validator.isValidNodeName(123), false);
      assert.strictEqual(rclnodejs.validator.isValidNodeName(null), false);
    });
  });

  describe('isValidTopicName', function () {
    it('should return true for valid topic names', function () {
      assert.strictEqual(rclnodejs.validator.isValidTopicName('chatter'), true);
      assert.strictEqual(
        rclnodejs.validator.isValidTopicName('{node}/chatter'),
        true
      );
      assert.strictEqual(
        rclnodejs.validator.isValidTopicName('~/chatter'),
        true
      );
      assert.strictEqual(
        rclnodejs.validator.isValidTopicName('/my/topic'),
        true
      );
    });

    it('should return false for invalid topic names', function () {
      assert.strictEqual(rclnodejs.validator.isValidTopicName(''), false);
      assert.strictEqual(
        rclnodejs.validator.isValidTopicName('/invalid_topic?'),
        false
      );
      assert.strictEqual(
        rclnodejs.validator.isValidTopicName('invalid/42topic'),
        false
      );
    });

    it('should return false for non-string input', function () {
      assert.strictEqual(rclnodejs.validator.isValidTopicName(123), false);
    });
  });

  describe('isValidNamespace', function () {
    it('should return true for valid namespaces', function () {
      assert.strictEqual(rclnodejs.validator.isValidNamespace('/my_ns'), true);
      assert.strictEqual(rclnodejs.validator.isValidNamespace('/'), true);
      assert.strictEqual(
        rclnodejs.validator.isValidNamespace('/deep/namespace'),
        true
      );
    });

    it('should return false for invalid namespaces', function () {
      assert.strictEqual(rclnodejs.validator.isValidNamespace(''), false);
      assert.strictEqual(
        rclnodejs.validator.isValidNamespace('invalid_namespace'),
        false
      );
      assert.strictEqual(
        rclnodejs.validator.isValidNamespace('invalid namespace'),
        false
      );
    });

    it('should return false for non-string input', function () {
      assert.strictEqual(rclnodejs.validator.isValidNamespace(123), false);
      assert.strictEqual(rclnodejs.validator.isValidNamespace(null), false);
    });
  });
});
