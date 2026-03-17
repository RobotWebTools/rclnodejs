// Copyright (c) 2026, The Robot Web Tools Contributors
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

/**
 * @class MessageInfo
 *
 * Contains metadata about a received message, including timestamps,
 * sequence numbers, and the publisher's globally unique identifier (GID).
 *
 * This is the rclnodejs equivalent of rclpy's MessageInfo.
 * It is passed as the second argument to subscription callbacks when the
 * callback accepts two parameters.
 *
 * @example
 * node.createSubscription(
 *   'std_msgs/msg/String',
 *   'topic',
 *   (msg, messageInfo) => {
 *     console.log('Source timestamp:', messageInfo.sourceTimestamp);
 *     console.log('Received at:', messageInfo.receivedTimestamp);
 *     console.log('Publisher GID:', messageInfo.publisherGid);
 *   }
 * );
 */
class MessageInfo {
  /**
   * Create a MessageInfo from a raw info object returned by the native layer.
   *
   * @param {object} rawInfo - Raw message info from rclTakeWithInfo
   * @hideconstructor
   */
  constructor(rawInfo) {
    /**
     * The timestamp when the message was published (nanoseconds since epoch).
     * @type {bigint}
     */
    this.sourceTimestamp = rawInfo.source_timestamp;

    /**
     * The timestamp when the message was received by the subscription (nanoseconds since epoch).
     * @type {bigint}
     */
    this.receivedTimestamp = rawInfo.received_timestamp;

    /**
     * The publication sequence number assigned by the publisher.
     * @type {bigint}
     */
    this.publicationSequenceNumber = rawInfo.publication_sequence_number;

    /**
     * The reception sequence number assigned by the subscriber.
     * @type {bigint}
     */
    this.receptionSequenceNumber = rawInfo.reception_sequence_number;

    /**
     * The globally unique identifier (GID) of the publisher.
     * A Buffer containing the raw GID bytes.
     * @type {Buffer}
     */
    this.publisherGid = rawInfo.publisher_gid;
  }

  /**
   * Convert to a plain object representation.
   *
   * @returns {object} Plain object with all metadata fields
   */
  toPlainObject() {
    return {
      sourceTimestamp: this.sourceTimestamp,
      receivedTimestamp: this.receivedTimestamp,
      publicationSequenceNumber: this.publicationSequenceNumber,
      receptionSequenceNumber: this.receptionSequenceNumber,
      publisherGid: this.publisherGid,
    };
  }
}

module.exports = MessageInfo;
