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

describe('rclnodejs validator testing', function() {
  this.timeout(60 * 1000);

  before(function() {
    return rclnodejs.init();
  });

  after(function() {
    rclnodejs.shutdown();
  });

  it('validate_full_topic_name', function() {
    var names = ['/chatter', '/node_name/chatter', '/ns/node_name/chatter'];
    names.forEach(name => {
      rclnodejs.validator.validateFullTopicName(name);
    });
  });

  it('validate_full_topic_name_failure_invalid_char', function() {
    utils.assertThrowsError(
      () => {
        rclnodejs.validator.validateFullTopicName('/invalid_topic?');
      },
      Error,
      'must not contain characters',
      'invalid full topic name!'
    );
  });

  it('validate_full_name_failure_not_absolute', function() {
    utils.assertThrowsError(
      () => {
        rclnodejs.validator.validateFullTopicName('invalid_topic');
      },
      Error,
      'must be absolute',
      'invalid full topic name!'
    );
  });

  it('validate_node_name', function() {
    rclnodejs.validator.validateNodeName('my_node');
  });

  it('validate_node_name_failures', function() {
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

  it('topic_or_service_is_hidden', function() {
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

    tests.forEach(test => {
      assert.deepStrictEqual(
        rclnodejs.isTopicOrServiceHidden(test[0]),
        test[1]
      );
    });
  });

  it('validate_topic_name', function() {
    var names = ['chatter', '{node}/chatter', '~/chatter'];

    names.forEach(name => {
      rclnodejs.validator.validateTopicName(name);
    });
  });

  it('valid_topic_name_failure_invalid_char', function() {
    utils.assertThrowsError(
      () => {
        rclnodejs.validator.validateTopicName('/invalid_topic?');
      },
      Error,
      'must not contain characters',
      'invalid topic name!'
    );
  });

  it('validate_topic_name_failure_start', function() {
    utils.assertThrowsError(
      () => {
        rclnodejs.validator.validateTopicName('invalid/42topic');
      },
      Error,
      'must not start with a number',
      'invalid topic name!'
    );
  });

  it('validate_namespace', function() {
    var names = ['/my_ns', '/'];

    names.forEach(name => {
      rclnodejs.validator.validateNamespace(name);
    });
  });

  it('validate_namespace_failures', function() {
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
});
