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

const assert = require('assert');
const rclnodejs = require('../index.js');
const { serializeMessage, deserializeMessage } = require('../index.js');

describe('rclnodejs publisher test suite', function () {
  [
    {
      type: 'std_msgs/msg/String',
      value: 'Hello ROS 2.0 Serialization!',
    },
    {
      type: 'std_msgs/msg/MultiArrayDimension',
      value: { label: 'label name 0', size: 256, stride: 4 },
    },
    {
      type: 'geometry_msgs/msg/Point',
      value: { x: 1.5, y: 2.75, z: 3.0 },
    },
  ].forEach((testCase) => {
    it('Test serialize a message of type ' + testCase.type, function () {
      const MyMessage = rclnodejs.require(testCase.type);
      const rosMsg = new MyMessage(testCase.value);
      const buffer = serializeMessage(rosMsg, MyMessage);

      assert(buffer instanceof Buffer);
      const deserializedRosMsg = deserializeMessage(buffer, MyMessage);
      assert.deepStrictEqual(
        deserializedRosMsg.toPlainObject(),
        rosMsg.toPlainObject()
      );
    });
  });
});
