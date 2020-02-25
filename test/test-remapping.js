// Copyright (c) Wayne Parrott All rights reserved.
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

describe('rcl node remapping', function() {
  let node;
  this.timeout(60 * 1000);

  afterEach(function() {
    if (node) node.destroy();
    rclnodejs.shutdown();
  });

  it('remap node name', async function() {
    const ARGV = ['--ros-args', '-r', 'my_node:__node:=foo'];

    await rclnodejs.init(rclnodejs.Context.defaultContext(), ARGV);

    node = rclnodejs.createNode('my_node');

    assert.equal(node.name(), 'foo');
  });

  it('remap node namespace', async function() {
    const ARGV = ['--ros-args', '-r', 'my_node:__ns:=/foo'];

    await rclnodejs.init(rclnodejs.Context.defaultContext(), ARGV);
    node = rclnodejs.createNode('my_node', '/my_ns');

    assert.equal(node.namespace(), '/foo');
  });

  it('remap publisher topic', async function() {
    const ARGV = ['--ros-args', '-r', 'my_node:/my_topic:=/foo_topic'];

    await rclnodejs.init(rclnodejs.Context.defaultContext(), ARGV);
    node = rclnodejs.createNode('my_node');
    let publisher = node.createPublisher('std_msgs/msg/String', 'my_topic');

    assert.equal(publisher.topic, 'foo_topic');
  });

  it('remap service name', async function() {
    const ARGV = ['--ros-args', '-r', 'my_node:my_service:=foo_service'];

    await rclnodejs.init(rclnodejs.Context.defaultContext(), ARGV);
    node = rclnodejs.createNode('my_node');
    let service = node.createService(
      'example_interfaces/srv/AddTwoInts',
      'my_service',
      request => {}
    );

    assert.equal(service.serviceName, 'foo_service');
  });

  it('remap node, namespace, topic and servicename', async function() {
    const ARGV = [
      '--ros-args',
      '-r',
      'my_node:__node:=foo_node',
      '-r',
      'foo_node:__ns:=/foo_ns',
      '-r',
      'foo_node:/my_topic:=foo_topic',
      '-r',
      'foo_node:my_service:=foo_service',
    ];

    await rclnodejs.init(rclnodejs.Context.defaultContext(), ARGV);

    node = rclnodejs.createNode('my_node', '/my_ns');
    let publisher = node.createPublisher('std_msgs/msg/String', '/my_topic');
    let service = node.createService(
      'example_interfaces/srv/AddTwoInts',
      'my_service',
      request => {}
    );

    assert.equal(node.name(), 'foo_node');
    assert.equal(node.namespace(), '/foo_ns');
    assert.equal(publisher.topic, 'foo_topic');
    assert.equal(service.serviceName, 'foo_service');
  });
});
