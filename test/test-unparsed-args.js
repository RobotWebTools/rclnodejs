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

import assert from 'assert';
import rclnodejs from '../index.js';

describe('rclnodejs unparsed ROS args', function () {
  this.timeout(10 * 1000);

  beforeEach(function () {
    rclnodejs.shutdown();
  });

  afterEach(function () {
    rclnodejs.shutdown();
  });

  it('should throw error when initializing context with unknown ROS args', async function () {
    const args = ['--ros-args', '--unknown-arg'];
    try {
      await rclnodejs.init(rclnodejs.Context.defaultContext(), args);
      assert.fail('Should have thrown an error');
    } catch (e) {
      assert.ok(e.message.includes('Unknown ROS arguments'));
      assert.ok(e.message.includes('--unknown-arg'));
    }
  });

  it('should throw error when creating node with unknown ROS args', async function () {
    await rclnodejs.init();
    const args = ['--ros-args', '--unknown-arg'];
    try {
      rclnodejs.createNode(
        'test_node',
        '',
        rclnodejs.Context.defaultContext(),
        rclnodejs.NodeOptions.defaultOptions,
        args
      );
      assert.fail('Should have thrown an error');
    } catch (e) {
      assert.ok(e.message.includes('Unknown ROS arguments'));
      assert.ok(e.message.includes('--unknown-arg'));
    }
  });
});
