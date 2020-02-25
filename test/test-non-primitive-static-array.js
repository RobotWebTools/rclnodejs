// Copyright (c) 2018 Intel Corporation. All rights reserved.
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

/* eslint-disable camelcase */
/* eslint-disable key-spacing */
/* eslint-disable comma-spacing */

describe('Test message which has a static non-primitive array', function() {
  this.timeout(60 * 1000);

  let time = {
    sec: 123456,
    nanosec: 789,
  };
  let time_value = [];
  time_value.push(time);
  time_value.push(time);

  beforeEach(function() {
    return rclnodejs.init();
  });

  afterEach(function() {
    rclnodejs.shutdown();
  });

  it('Assigned with an array whose length is 2', function(done) {
    const node = rclnodejs.createNode('publish_time');
    let publisher = node.createPublisher(
      'rclnodejs_test_msgs/msg/StaticArrayNonPrimitives',
      'time'
    );
    let timer = setInterval(() => {
      assert.doesNotThrow(() => {
        console.log('length is ' + time_value.length);
        publisher.publish({ time_value });
      }, RangeError);
    }, 100);

    node.createSubscription(
      'rclnodejs_test_msgs/msg/StaticArrayNonPrimitives',
      'time',
      msg => {
        clearInterval(timer);
        assert.deepStrictEqual(msg.time_value, time_value);
        node.destroy();
        done();
      }
    );

    rclnodejs.spin(node);
  });

  it('Assigned with an array whose length is greater than 2', function(done) {
    time_value.push(time);
    const node = rclnodejs.createNode('publish_time');
    let publisher = node.createPublisher(
      'rclnodejs_test_msgs/msg/StaticArrayNonPrimitives',
      'time'
    );
    assert.throws(() => {
      publisher.publish({ time_value });
    }, RangeError);
    node.destroy();
    done();
  });

  it('Assigned with an array whose length is less than 2', function(done) {
    const node = rclnodejs.createNode('publish_time');
    let publisher = node.createPublisher(
      'rclnodejs_test_msgs/msg/StaticArrayNonPrimitives',
      'time'
    );
    assert.throws(() => {
      publisher.publish({ time_value: time_value.slice(0, 0) });
    }, RangeError);
    node.destroy();
    done();
  });
});
