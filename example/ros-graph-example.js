// Copyright (c) 2021 Wayne Parrott, All rights reserved.
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

const rclnodejs = require('../index.js');

console.log(
  'This example creates the following nodes and outputs the corresponding ROS2 graph:'
);
console.log('  publisher_node');
console.log('  subscriber_node');
console.log('  service_node');
console.log('  ros_graph_display_node');

rclnodejs
  .init()
  .then(() => {
    const ns = 'ns1';

    let publisherNode = new rclnodejs.Node('publisher_node', ns);
    publisherNode.createPublisher('std_msgs/msg/String', 'topic');

    let subscriberNode = new rclnodejs.Node('subscriber_node', ns);
    subscriberNode.createSubscription('std_msgs/msg/String', 'topic', () => {});

    let serviceNode = new rclnodejs.Node('service_node', ns);
    serviceNode.createService(
      'example_interfaces/srv/AddTwoInts',
      'add_two_ints',
      (request, response) => {
        let result = response.template;
        result.sum = request.a + request.b;
        response.send(result);
      }
    );

    let node = rclnodejs.createNode('ros_graph_display_node', ns);
    let nodeNamesAndNameSpaces = node.getNodeNamesAndNamespaces();

    console.log('NODES');
    console.log(node.getNodeNames());
    console.log(nodeNamesAndNameSpaces);

    console.log('TOPICS & TYPES');
    console.log(node.getTopicNamesAndTypes());

    console.log('SERVICES & TYPES');
    console.log(node.getServiceNamesAndTypes());

    console.log('PUBLISHERS BY NODE');
    console.log(
      JSON.stringify(
        nodeNamesAndNameSpaces.map((nameNs) => {
          return {
            node: {
              name: nameNs.name,
              namespace: nameNs.namespace,
            },
            info: node.getPublisherNamesAndTypesByNode(
              nameNs.name,
              nameNs.namespace
            ),
          };
        }),
        undefined,
        '  '
      )
    );

    console.log('SUBSCRIPTIONS BY NODE');
    console.log(
      JSON.stringify(
        nodeNamesAndNameSpaces.map((nameNs) => {
          return {
            node: {
              name: nameNs.name,
              namespace: nameNs.namespace,
            },
            info: node.getSubscriptionNamesAndTypesByNode(
              nameNs.name,
              nameNs.namespace
            ),
          };
        }),
        undefined,
        '  '
      )
    );

    console.log('SERVICES BY NODE');
    console.log(
      JSON.stringify(
        nodeNamesAndNameSpaces.map((nameNs) => {
          return {
            node: {
              name: nameNs.name,
              namespace: nameNs.namespace,
            },
            info: node.getServiceNamesAndTypesByNode(
              nameNs.name,
              nameNs.namespace
            ),
          };
        }),
        undefined,
        '  '
      )
    );
  })
  .catch((e) => {
    console.log(`Error: ${e}`);
  });
