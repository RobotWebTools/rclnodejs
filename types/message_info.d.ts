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

declare module 'rclnodejs' {
  /**
   * Contains metadata about a received message, including timestamps,
   * sequence numbers, and the publisher's globally unique identifier (GID).
   *
   * Passed as the second argument to subscription callbacks when the
   * callback accepts two parameters.
   *
   * @example
   * ```typescript
   * node.createSubscription(
   *   'std_msgs/msg/String',
   *   'topic',
   *   (msg: rclnodejs.std_msgs.msg.String, info: MessageInfo) => {
   *     console.log('Source timestamp:', info.sourceTimestamp);
   *   }
   * );
   * ```
   */
  class MessageInfo {
    /**
     * The timestamp when the message was published (nanoseconds since epoch).
     */
    readonly sourceTimestamp: bigint;

    /**
     * The timestamp when the message was received by the subscription (nanoseconds since epoch).
     */
    readonly receivedTimestamp: bigint;

    /**
     * The publication sequence number assigned by the publisher.
     */
    readonly publicationSequenceNumber: bigint;

    /**
     * The reception sequence number assigned by the subscriber.
     */
    readonly receptionSequenceNumber: bigint;

    /**
     * The globally unique identifier (GID) of the publisher.
     * A Buffer containing the raw GID bytes.
     */
    readonly publisherGid: Buffer;

    /**
     * Convert to a plain object representation.
     */
    toPlainObject(): {
      sourceTimestamp: bigint;
      receivedTimestamp: bigint;
      publicationSequenceNumber: bigint;
      receptionSequenceNumber: bigint;
      publisherGid: Buffer;
    };
  }
}
