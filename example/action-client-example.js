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
    const GoalStatus = rclnodejs.require('actionlib_msgs/msg/GoalStatus');

    const ac = new rclnodejs.ActionClientInterface({
      type: 'ros1_actions/msg/DoDishes',
      actionServer: 'dishes',
      rclnodejs: rclnodejs,
    });

    let goal = ac.sendGoal({ goal: { dishwasher_id: 1 } });
    console.log(`The goal was sent, the goal id is ${goal.goal_id.id}`);

    ac.on('feedback', feedback => {
      console.log(
        `${feedback.percent_complete}% of the task has been completed.`
      );
    });

    ac.on('status', status => {
      status.status_list.forEach(s => {
        if (
          s.goal_id.id === goal.goal_id.id &&
          s.status === GoalStatus.SUCCEEDED
        ) {
          console.log(
            `The goal, whose id is ${s.goal_id.id}, has been executed successfully.`
          );
        }
      });
    });

    ac.on('result', result => {
      if (result) {
        console.log(`${result.total_dishes_cleaned} dishes have been cleaned.`);
      }
    });
  })
  .catch(err => {
    console.error(err);
  });
