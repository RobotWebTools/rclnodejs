// Copyright (c) 2020 Intel Corporation. All rights reserved.
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

const childProcess = require('child_process');
const assert = require('assert');
const rclnodejs = require('../index.js');
const DistroUtils = rclnodejs.DistroUtils;

describe('rclnodejs publisher test suite', function () {
  this.timeout(60 * 1000);

  beforeEach(function () {
    return rclnodejs.init();
  });

  afterEach(function () {
    rclnodejs.shutdown();
  });

  it('Publish raw serialized messages', function (done) {
    const node = rclnodejs.createNode('serialized_messages_node');
    const topic = Buffer.from('Hello ROS World');
    const publisher = node.createPublisher(
      'test_msgs/msg/BasicTypes',
      'chatter'
    );
    let timer = setInterval(function () {
      publisher.publish(topic);
    }, 100);

    node.createSubscription(
      'test_msgs/msg/BasicTypes',
      'chatter',
      { isRaw: true },
      (msg) => {
        clearInterval(timer);

        let buffer = topic;
        const distroId = DistroUtils.getDistroId();

        if (distroId >= DistroUtils.DistroId.GALACTIC) {
          // The received Buffer is null-terminated.
          buffer = Buffer.concat([buffer, Buffer.from([0x00])]);
        }
        assert.deepStrictEqual(
          Buffer.compare(msg, buffer),
          0,
          `distro: ${DistroUtils.getDistroName()}, buffer: ${buffer}, msg: ${msg}`
        );
        done();
      }
    );

    rclnodejs.spin(node);
  });
});
