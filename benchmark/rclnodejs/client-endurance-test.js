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

const rclnodejs = require('../../index.js');
const readline = require('readline');

const rl = readline.createInterface({
  input: process.stdin,
  output: process.stdout
});

rl.question('How many times do you want to run? ', (times) => {
  rclnodejs.init().then(() => {
    console.log(`The client will send a SetBool request continuously until receiving response ${times} times.`);
    console.log(`Begin at ${new Date().toString()}.`);

    const node = rclnodejs.createNode('endurance_client_rclnodejs');
    const client = node.createClient('std_srvs/srv/SetBool', 'set_flag');
    let sentTimes = 0;
    let receivedTimes = 0;
    let totalTimes = parseInt(times, 10);
    let sendRequest = function(response) {
      client.sendRequest(true, (response) => {
        if (++receivedTimes > totalTimes) {
          rclnodejs.shutdown();
          console.log(`End at ${new Date().toString()}`);
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

  rl.close();
});
