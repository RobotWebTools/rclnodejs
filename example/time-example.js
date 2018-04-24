// Copyright (c) 2017 Intel Corporation. All rights reserved.
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

// Use system clock to get time (in nanosecond precision)
//  For clock types, see http://design.ros2.org/articles/clock_and_time.html
const clock = new rclnodejs.Time('system-clock');

// Get time and output it
const start = clock.now();
console.log('Start:', start);

// Do something with time cost
for (let i = 0; i <= 314; ++ i) {
  process.stdout.write('.');
}
process.stdout.write('\n');

// Get finish time
const finish = clock.now();
console.log('Finish: ', finish);
process.stdout.write('\n');

// Calculate elapsed time span
const delta = rclnodejs.Time.diff(start, finish);

// Format string and output it
let text = `Time elapsed: ${delta.sec} second`;
text += ` ${Math.floor(delta.nanosec / 1000 / 1000)} ms`;
delta.nanosec %= 1000 * 1000;
text += ` ${Math.floor(delta.nanosec / 1000)} us`;
delta.nanosec %= 1000;
text += ` ${delta.nanosec} ns`;
console.log(text);
