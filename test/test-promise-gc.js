/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

/* eslint-disable camelcase */
'use strict';

const assert = require('assert');
const rclnodejs = require('../index.js');

/**
 * These tests need to be run with `--expose-gc` flags.
 */
describe('Test promise wrapper with garbage collection', function () {
  this.timeout(60 * 1000);

  before(function () {
    return rclnodejs.init();
  });

  after(function () {
    rclnodejs.shutdown();
  });

  [
    {
      typeClass: 'std_msgs/msg/String',
      msg: {
        data: 'Hello World!',
      },
    },
    {
      typeClass: 'std_msgs/msg/UInt8MultiArray',
      msg: {
        layout: { dim: [{ label: 'test', size: 10, stride: 10 }], data_offset: 0 },
        data: Uint8Array.from([0, 1, 2, 3, 4, 5, 6, 7, 8, 9]),
      },
    },
    {
      typeClass: 'sensor_msgs/msg/JointState',
      msg: {
        header: {
          stamp: {
            sec: 123456,
            nanosec: 789,
          },
          frame_id: 'main frame',
        },
        name: ['Tom', 'Jerry'],
        position: Float64Array.from([1, 2]),
        velocity: Float64Array.from([2, 3]),
        effort: Float64Array.from([4, 5, 6]),
      },
    },
  ].forEach(({ typeClass, msg }, i) =>
    it(`${typeClass} message does not get garbage collected`, async function () {
      const node = rclnodejs.createNode(`promise_gc_${i}`);
      rclnodejs.spin(node);

      const topic = `promise_gc_channel_${i}`;

      const publisher = node.createPublisher(typeClass, topic);
      const timer = node.createTimer(100, () => publisher.publish(msg));

      const result = await new Promise((res) =>
        node.createSubscription(typeClass, topic, (msg) => {
          timer.cancel();
          node.destroy();
          res(msg);
        })
      );
      global.gc();
      assert.deepEqual(result, msg);
    })
  );
});
