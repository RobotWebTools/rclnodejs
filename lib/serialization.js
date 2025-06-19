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

const rclnodejs = require('bindings')('rclnodejs');

class Serialization {
  static serializeMessage(message, typeClass) {
    if (!(message instanceof typeClass)) {
      throw new TypeError('Message must be a valid ros2 message type');
    }
    return rclnodejs.serialize(
      typeClass.type().pkgName,
      typeClass.type().subFolder,
      typeClass.type().interfaceName,
      message.serialize()
    );
  }

  static deserializeMessage(buffer, typeClass) {
    if (!(buffer instanceof Buffer)) {
      throw new TypeError('Buffer is required for deserialization');
    }
    const rosMsg = new typeClass();
    rclnodejs.deserialize(
      typeClass.type().pkgName,
      typeClass.type().subFolder,
      typeClass.type().interfaceName,
      buffer,
      rosMsg.toRawROS()
    );
    return rosMsg;
  }
}

module.exports = Serialization;
