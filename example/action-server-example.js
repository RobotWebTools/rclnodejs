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

/* eslint-disable camelcase */

'use strict';

const rclnodejs = require('../index.js');

rclnodejs
  .init()
  .then(() => {
    const as = new rclnodejs.ActionLib.ActionServer({
      type: 'ros1_actions/msg/DoDishes',
      actionServer: 'dishes',
      rclnodejs: rclnodejs,
    });

    as.on('goal', function(goal) {
      console.log(`A goal, whose id is ${goal.getGoalId().id}, was received.`);

      goal.setAccepted('goal accepted');

      /*
    For the state transitions, please reference http://wiki.ros.org/actionlib/DetailedDescription,
    besides the setAccept operation, others are:
    goal.setCancelRequested();
    goal.setCancelled({total_dishes_cleaned: 10}, 'canceled');
    goal.setRejected({total_dishes_cleaned: 0}, 'rejected');
    goal.setAborted({total_dishes_cleaned: 0}, 'aborted');
    goal.setSucceeded({total_dishes_cleaned: 100}, 'done');
    */

      let feedback = {
        percent_complete: 70,
        image: {
          header: {
            stamp: {
              sec: 11223,
              nanosec: 44556,
            },
            frame_id: 'f001',
          },
          height: 240,
          width: 320,
          encoding: 'rgba',
          is_bigendian: false,
          step: 320 * 16,
          is_dense: false,
          data: Uint8Array.from({ length: 320 * 240 }, (v, k) => k),
        },
      };

      goal.publishFeedback(feedback);
      setTimeout(() => {
        goal.setSucceeded({ total_dishes_cleaned: 10 }, 'done');
      }, 500);
    });

    as.on('cancel', goalHandle => {
      console.log(
        `The goal, whose id is ${
          goalHandle.getGoalStatus().goal_id.id
        }, is canceled. `
      );
    });

    as.start();
  })
  .catch(err => {
    console.error(err);
  });
