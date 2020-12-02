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
      '/node_name/chatter': ['~/chatter', 'node_name', '/'],
    };

    for (let test in tests) {
      var [topic, name, namespace] = tests[test];
      assert.deepStrictEqual(
        test,
        rclnodejs.expandTopicName(topic, name, namespace)
      );
    }
  });

  it('expand_topic_name_invalid_node_name', function() {
    utils.assertThrowsError(
      () => {
        rclnodejs.expandTopicName('topic', 'invalid_node_name?', '/ns');
      },
      Error,
      'node name is invalid',
      'invalid node name'
    );
  });

  it('expand_topic_name_invalid_namespace_empty', function() {
    utils.assertThrowsError(
      () => {
        rclnodejs.expandTopicName('topic', 'node_name', '');
      },
      Error,
      'node namespace is invalid',
      'invalid namespace'
    );
  });

  it('expand_topic_name_invalid_namespace_relative', function() {
    utils.assertThrowsError(
      () => {
        rclnodejs.expandTopicName('topic', 'node_name', 'ns');
      },
      Error,
      'node namespace is invalid',
      'invalid namespace'
    );
  });

  it('expand_topic_name_invalid_topic', function() {
    utils.assertThrowsError(
      () => {
        rclnodejs.expandTopicName('invalid/topic?', 'node_name', '/ns');
      },
      Error,
      'topic name is invalid',
      'invalid topic name'
    );
  });
});

describe('rclnodejs topic string type coverage testing', function() {
  this.timeout(60 * 1000);

  before(function() {
    return rclnodejs.init();
  });

  after(function() {
    rclnodejs.shutdown();
  });

  let testData = [
    {
      title: ' name cannot be empty',
      nodeName: 'empty_',
      topicName: '',
      serviceName: '',
      matchMsg: 'topic name is invalid',
      msg: ' name should not be empty!',
    },
    {
      title: ' name cannot contain single quote',
      nodeName: 'single_quote_',
      topicName: "'single_quote_topic'",
      serviceName: "'single_quote_service'",
      matchMsg: 'topic name is invalid',
      msg: ' name should not be empty!',
    },
    {
      title: ' name cannot contain double quotes',
      nodeName: 'double_quotes_',
      topicName: '"double_quotes_topic"',
      serviceName: '"double_quotes_service"',
      matchMsg: 'topic name is invalid',
      msg: ' name should not contain double quotes!',
    },
    {
      title: ' name cannot be unicode',
      nodeName: 'unicode_',
      topicName: '\u8bdd\u9898',
      serviceName: '\u8bdd\u9898',
      matchMsg: 'topic name is invalid',
      msg: ' name cannot support unicode!',
    },
  ];

  testData.forEach((data, index) => {
    it('topic' + data.title, function() {
      var node = rclnodejs.createNode(data.nodeName + 'topic_node');

      utils.assertThrowsError(
        () => {
          var publisher = node.createPublisher(
            'std_msgs/msg/String',
            data.topicName
          );
        },
        Error,
        data.matchMsg,
        'topic' + data.msg
      );
      utils.assertThrowsError(
        () => {
          var publisher = node.createSubscription(
            'std_msgs/msg/String',
            data.topicName,
            msg => {}
          );
        },
        Error,
        data.matchMsg,
        'topic' + data.msg
      );
    });

    it('service' + data.title, function() {
      var node = rclnodejs.createNode(data.nodeName + 'service_node');

      utils.assertThrowsError(
        () => {
          var service = node.createService(
            'example_interfaces/srv/AddTwoInts',
            data.serviceName,
            (res, req) => {}
          );
        },
        Error,
        data.matchMsg,
        'service' + data.msg
      );
      utils.assertThrowsError(
        () => {
          var client = node.createClient(
            'example_interfaces/srv/AddTwoInts',
            data.serviceName
          );
        },
        Error,
        data.matchMsg,
        'service' + data.msg
      );
    });
  });
});
