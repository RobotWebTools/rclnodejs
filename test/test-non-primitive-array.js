// Copyright (c) 2018 Intel Corporation. All rights reserved.
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

const assert = require('assert');
const rclnodejs = require('../index.js');

/* eslint-disable camelcase */
/* eslint-disable key-spacing */
/* eslint-disable comma-spacing */

describe('Test message which has a non-primitive array', function () {
  this.timeout(60 * 1000);

  let joy_feedback = {
    type: 2, // TYPE_BUZZER
    id: 1,
    intensity: 101,
  };

  let joyfeedback_array = [];
  joyfeedback_array.push(joy_feedback);
  joyfeedback_array.push(joy_feedback);

  beforeEach(function () {
    return rclnodejs.init();
  });

  afterEach(function () {
    rclnodejs.shutdown();
  });

  it('Assigned with an array whose length is 2', function (done) {
    const node = rclnodejs.createNode('publish_time');
    let publisher = node.createPublisher(
      'sensor_msgs/msg/JoyFeedbackArray',
      'joyfeedback'
    );
    let timer = setInterval(() => {
      assert.doesNotThrow(() => {
        publisher.publish({ array: joyfeedback_array });
      }, RangeError);
    }, 100);

    node.createSubscription(
      'sensor_msgs/msg/JoyFeedbackArray',
      'joyfeedback',
      (msg) => {
        clearInterval(timer);
        assert.deepStrictEqual(msg.array, joyfeedback_array);
        node.destroy();
        done();
      }
    );

    rclnodejs.spin(node);
  });
});
