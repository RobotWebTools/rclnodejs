// Copyright (c) 2025, The Robot Web Tools Contributors
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

describe('rclnodejs subscription test suite', function () {
  this.timeout(60 * 1000);

  beforeEach(function () {
    return rclnodejs.init();
  });

  afterEach(function () {
    rclnodejs.shutdown();
  });

  it('Test count of subscription', function () {
    const node = rclnodejs.createNode('publisher_node');
    const String = 'std_msgs/msg/String';
    node.createPublisher(String, 'topic');
    const subscription = node.createSubscription(String, 'topic', (msg) => {});
    assert.strictEqual(subscription.publisherCount, 1);
  });

  it('Test loggerName', function () {
    const node = rclnodejs.createNode('publisher_node');
    const subscription = node.createSubscription(String, 'topic', (msg) => {});
    assert.strictEqual(typeof subscription.loggerName, 'string');
  });
});
