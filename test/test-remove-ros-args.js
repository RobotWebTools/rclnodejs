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

describe('rclnodejs removeROSArgs', function () {
  it('should remove ROS arguments', function () {
    const args = ['node', 'script.js', '--ros-args', '-r', '__node:=my_node'];
    const nonRosArgs = rclnodejs.removeROSArgs(args);
    assert.deepStrictEqual(nonRosArgs, ['node', 'script.js']);
  });

  it('should keep non-ROS arguments', function () {
    const args = ['node', 'script.js', 'arg1', 'arg2'];
    const nonRosArgs = rclnodejs.removeROSArgs(args);
    assert.deepStrictEqual(nonRosArgs, ['node', 'script.js', 'arg1', 'arg2']);
  });

  it('should handle mixed arguments', function () {
    const args = [
      'node',
      'script.js',
      'arg1',
      '--ros-args',
      '-r',
      '__node:=my_node',
      '--',
      'arg2',
    ];
    const nonRosArgs = rclnodejs.removeROSArgs(args);

    assert.ok(nonRosArgs.includes('arg1'));
    assert.ok(nonRosArgs.includes('arg2'));
    assert.ok(!nonRosArgs.includes('--ros-args'));
    assert.ok(!nonRosArgs.includes('__node:=my_node'));
  });

  it('should throw if argv is not an array', function () {
    assert.throws(() => {
      rclnodejs.removeROSArgs('not an array');
    }, TypeError);
  });

  it('should throw if argv elements are not strings', function () {
    assert.throws(() => {
      rclnodejs.removeROSArgs(['string', 123]);
    }, TypeError);
  });
});
