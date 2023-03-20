// Copyright (c) 2023 Wayne Parrott. All rights reserved.
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

const { assertDefined } = require('dtslint/bin/util.js');
const rclnodejs = require('../index.js');

/**
 * This example demonstrates the use of content-filtering
 * topics (subscriptions) that were introduced in ROS 2 Humble.
 * See the following resources for content-filtering in ROS:
 * @see {@link Node#options}
 * @see {@link Node#createSubscription}
 * @see {@link https://www.omg.org/spec/DDS/1.4/PDF|DDS 1.4 specification, Annex B}
 *
 * Use publisher-content-filter-example.js to generate example messages.
 *
 * To see all published messages (filterd + unfiltered) run this
 * from commandline:
 *
 *   ros2 topic echo temperature
 *
 * @return {undefined}
 */
async function main() {
  await rclnodejs.init();
  const node = new rclnodejs.Node('subscription_message_example_node');

  let param = 50;

  // create a content-filter to limit incoming messages to
  // only those with temperature > paramC.
  const options = rclnodejs.Node.getDefaultOptions();
  options.contentFilter = {
    expression: 'temperature > %0',
    parameters: [param],
  };

  let count = 0;
  let subscription;
  try {
    subscription = node.createSubscription(
      'sensor_msgs/msg/Temperature',
      'temperature',
      options,
      (temperatureMsg) => {
        console.log(`Received temperature message-${++count}: 
${temperatureMsg.temperature}C`);
        if (count % 5 === 0) {
          if (subscription.hasContentFilter()) {
            console.log('Clearing filter');
            subscription.clearContentFilter();
          } else if (param < 100) {
            param += 10;
            console.log('Update topic content-filter, temperature > ', param);
            const contentFilter = {
              expression: 'temperature > %0',
              parameters: [param],
            };
            subscription.setContentFilter(contentFilter);
          }
          console.log(
            'Content-filtering enabled: ',
            subscription.hasContentFilter()
          );
        }
      }
    );

    if (!subscription.hasContentFilter()) {
      console.log('Content-filtering is not enabled on subscription.');
    }
  } catch (error) {
    console.error('Unable to create content-filtering subscription.');
    console.error(
      'Please ensure your content-filter expression and parameters are well-formed.'
    );
  }

  node.spin();
}

main();
