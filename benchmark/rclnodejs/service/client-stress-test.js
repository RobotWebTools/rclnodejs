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

/* eslint-disable camelcase */
const app = require('commander');
const rclnodejs = require('../../../index.js');

app
  .option('-r, --run <n>', 'How many times to run')
  .parse(process.argv);

rclnodejs.init().then(() => {
  const time = process.hrtime();
  const node = rclnodejs.createNode('stress_client_rclnodejs');
  const client = node.createClient('nav_msgs/srv/GetMap', 'get_map');
  let receivedTimes = 0;
  let totalTimes = app.run || 1;
  console.log('The client will send a GetMap request continuously' +
    ` until receiving response ${totalTimes} times.`);
  let sendRequest = function() {
    client.sendRequest({}, (response) => {
      if (++receivedTimes > totalTimes) {
        rclnodejs.shutdown();
        const diff = process.hrtime(time);
        console.log(`Benchmark took ${diff[0]} seconds and ${Math.ceil(diff[1] / 1000000)} milliseconds.`);
      } else {
        setImmediate(sendRequest);
      }
    });
  };

  sendRequest();
  rclnodejs.spin(node);
}).catch((e) => {
  console.log(`Error: ${e}`);
});
