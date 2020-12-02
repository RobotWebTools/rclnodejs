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
const childProcess = require('child_process');
const rclnodejs = require('../index.js');

describe('rclnodejs message communication', function() {
  this.timeout(60 * 1000);

  before(function() {
    return rclnodejs.init();
  });

  after(function() {
    rclnodejs.shutdown();
  });

  it('should support array type', function(done) {
    var node = rclnodejs.createNode('array_message_subscription');
    const JointState = 'sensor_msgs/msg/JointState';
    var publisher = childProcess.fork(`${__dirname}/publisher_array_setup.js`);
    var destroy = false;
    var subscription = node.createSubscription(
      JointState,
      'JointState',
      state => {
        assert.deepStrictEqual(state.header.stamp.sec, 123456);
        assert.deepStrictEqual(state.header.stamp.nanosec, 789);
        assert.deepStrictEqual(state.header.frame_id, 'main frame');
        assert.deepStrictEqual(state.name, ['Tom', 'Jerry']);
        assert.deepStrictEqual(state.position, Float64Array.from([1, 2]));
        assert.deepStrictEqual(state.velocity, Float64Array.from([2, 3]));
        assert.deepStrictEqual(state.effort, Float64Array.from([4, 5, 6]));

        if (!destroy) {
          publisher.kill('SIGINT');
          node.destroy();
          destroy = true;
          done();
        }
      }
    );
    rclnodejs.spin(node);
  });
});
