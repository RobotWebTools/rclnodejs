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

const rclnodejs = require('../index.js');

/**
 * This example demonstrates a rate limited loop running at
 * 0.5 hz (once every 2 secs) and a publisher sending messages
 * every 10 ms from an setInterval(). Thus, the subscriber is
 * receiving only every 200th published message.
 *
 * To see every published message run this from commandline:
 *   > ros2 topic echo topic 'std_msgs/msg/String'
 *
 * @return {undefined}
 */
async function main() {
  await rclnodejs.init();
  const node = rclnodejs.createNode('test_node');
  const publisher = node.createPublisher('std_msgs/msg/String', 'topic');
  const subscriptions = node.createSubscription(
    'std_msgs/msg/String',
    'topic',
    undefined,
    msg => console.log(`Received(${Date.now()}): ${msg.data}`)
  );
  const rate = node.createRate(0.5);

  setInterval(() => publisher.publish(`hello ${Date.now()}`), 10);

  let forever = true;
  while (forever) {
    await rate.sleep();
    rclnodejs.spinOnce(node, 1000);
  }
}

main();
