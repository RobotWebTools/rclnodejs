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

describe('rclnodejs expand topic API testing', function() {
  this.timeout(60 * 1000);

  before(function() {
    return rclnodejs.init();
  });

  after(function() {
    rclnodejs.shutdown();
  });

  it('test_expand_topic_name', function() {
    var tests = {
      '/ns/chatter': ['chatter', 'node_name', '/ns'],
      '/chatter': ['chatter', 'node_name', '/'],
      '/ns/node_name/chatter': ['~/chatter', 'node_name', '/ns'],
      '/node_name/chatter': ['~/chatter', 'node_name', '/']
    };

    for (let test in tests) {
      var [topic, name, namespace] = tests[test];
      assert.deepStrictEqual(test, rclnodejs.expandTopicName(topic, name, namespace));
    }
  });

  it('expand_topic_name_invalid_node_name', function() {
    utils.assertThrowsError(() => {
      rclnodejs.expandTopicName('topic', 'invalid_node_name?', '/ns');
    }, Error, 'node name is invalid', 'invalid node name');
  });

  it('expand_topic_name_invalid_namespace_empty', function() {
    utils.assertThrowsError(() => {
      rclnodejs.expandTopicName('topic', 'node_name', '');
    }, Error, 'node namespace is invalid', 'invalid namespace');
  });

  it('expand_topic_name_invalid_namespace_relative', function() {
    utils.assertThrowsError(() => {
      rclnodejs.expandTopicName('topic', 'node_name', 'ns');
    }, Error, 'node namespace is invalid', 'invalid namespace');
  });

  it('expand_topic_name_invalid_topic', function() {
    utils.assertThrowsError(() => {
      rclnodejs.expandTopicName('invalid/topic?', 'node_name', '/ns');
    }, Error, 'topic name is invalid', 'invalid topic name');
  });
});
