// Copyright (c) 2020 Matt Richard. All rights reserved.
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

const ActionInterfaces = require('./interfaces.js');
const { v4: uuidv4 } = require('uuid');

/**
 * @class - Represents a unique identifier used by actions.
 * @ignore
 */
class ActionUuid {
  /**
   * Creates a new instance of ActionUuid.
   * @constructor
   *
   * @param {Uint8Array} bytes - The bytes to create the UUID from. A new random UUID will be created, if not provided.
   */
  constructor(bytes) {
    if (bytes) {
      if (!(bytes instanceof Uint8Array)) {
        throw new Error('Invalid parameter');
      }

      this._bytes = bytes;
    } else {
      // Generate random UUID.
      let uuid = uuidv4().replace(/-/g, '');
      this._bytes = Uint8Array.from(Buffer.from(uuid, 'hex'));
    }
  }

  /**
   * Creates a new {@link ActionUuid} from the given bytes.
   * @param {Uint8Array} bytes - The bytes to create the UUID from.
   * @returns {ActionUuid} - The new UUID.
   */
  static fromBytes(bytes) {
    return new ActionUuid(bytes);
  }

  /**
   * Creates a new random {@link ActionUuid}.
   * @returns {ActionUuid} - The new UUID.
   */
  static random() {
    return new ActionUuid();
  }

  /**
   * Gets the bytes forming the UUID.
   */
  get bytes() {
    return this._bytes;
  }

  /**
   * Returns the UUID as a string.
   * @returns {string} - String representation of the UUID.
   */
  toString() {
    return [].slice.call(this._bytes).join(',');
  }

  /**
   * Create an instance from a ROS2 UUID message
   * @param {unique_identifier_msgs.msg.UUID} msg - The ROS2 UUID message
   * @returns {ActionUuid} - The new instance.
   */
  static fromMessage(msg) {
    return ActionUuid.fromBytes(msg.uuid);
  }

  /**
   * Create a ROS2 UUID message from this instance.
   * @returns {unique_identifier_msgs.msg.UUID} - The new ROS2 UUID message
   */
  toMessage() {
    const uuidMsg = new ActionInterfaces.UUID();
    uuidMsg.uuid = this.bytes;
    return uuidMsg;
  }

  /**
   * Create a ROS2 UUID message with random uuid data
   * @returns {ActionUuid} - The new instance.
   */
  static randomMessage() {
    return ActionUuid.random().toMessage();
  }
}

module.exports = ActionUuid;
