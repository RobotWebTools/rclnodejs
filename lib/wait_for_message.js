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

const { TimeoutError } = require('./errors.js');

/**
 * Wait for a single message on a topic.
 *
 * Creates a temporary subscription, waits for the first message to arrive,
 * and returns it. The temporary subscription is always cleaned up, even on
 * timeout or error. The node must be spinning before calling this function.
 *
 * This is the rclnodejs equivalent of rclpy's `wait_for_message`.
 *
 * @param {function|string|object} typeClass - The ROS message type class.
 * @param {Node} node - The node to create the temporary subscription on.
 * @param {string} topic - The topic name to listen on.
 * @param {object} [options] - Options.
 * @param {number} [options.timeout] - Timeout in milliseconds. If omitted, waits indefinitely.
 * @param {object} [options.qos] - QoS profile for the subscription.
 * @returns {Promise<object>} - Resolves with the received message.
 * @throws {Error} If timeout expires before a message arrives.
 *
 * @example
 * node.spin();
 * const msg = await waitForMessage(
 *   'std_msgs/msg/String',
 *   node,
 *   '/my_topic',
 *   { timeout: 5000 }
 * );
 * console.log('Received:', msg.data);
 */
function waitForMessage(typeClass, node, topic, options = {}) {
  return new Promise((resolve, reject) => {
    let subscription = null;
    let timer = null;
    let settled = false;

    const cleanup = () => {
      if (timer) {
        clearTimeout(timer);
        timer = null;
      }
      if (subscription) {
        try {
          node.destroySubscription(subscription);
        } catch {
          // Subscription may already be destroyed if node is shutting down
        }
        subscription = null;
      }
    };

    const settle = (err, msg) => {
      if (settled) return;
      settled = true;
      cleanup();
      if (err) {
        reject(err);
      } else {
        resolve(msg);
      }
    };

    try {
      const subOptions = {};
      if (options.qos) {
        subOptions.qos = options.qos;
      }

      subscription = node.createSubscription(
        typeClass,
        topic,
        subOptions,
        (msg) => {
          settle(null, msg);
        }
      );

      if (options.timeout != null && options.timeout >= 0) {
        timer = setTimeout(() => {
          settle(
            new TimeoutError(
              `waitForMessage timed out after ${options.timeout}ms on topic '${topic}'`,
              { entityType: 'topic', entityName: topic }
            )
          );
        }, options.timeout);
      }
    } catch (err) {
      cleanup();
      reject(err);
    }
  });
}

module.exports = waitForMessage;
