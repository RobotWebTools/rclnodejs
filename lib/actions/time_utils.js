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

const NSEC_TO_SEC = 1e-9;
const USEC_TO_SEC = 1e-6;
const MSEC_TO_SEC = 1e-3;

module.exports = {
  rosTimeToDate(rosTime) {
    let date = new Date();
    // setTime takes in ms since epoch
    date.setTime(
      rosTime.sec * 1000 + Math.floor(rosTime.nanosec * USEC_TO_SEC)
    );
    return date;
  },

  dateToRosTime(date) {
    let sec = Math.floor(date * MSEC_TO_SEC);
    let nanosec = (date % 1000) * 1000000;
    return { sec, nanosec };
  },

  now() {
    return this.dateToRosTime(Date.now());
  },

  epoch() {
    return {
      sec: 0,
      nanosec: 0,
    };
  },

  isZeroTime(rosTime) {
    return rosTime.sec === 0 && rosTime.nanosec === 0;
  },

  toNumber(rosTime) {
    return this.toSeconds(rosTime);
  },

  toSeconds(rosTime) {
    return rosTime.sec + rosTime.nanosec * NSEC_TO_SEC;
  },

  timeComp(left, right) {
    return Math.sign(this.toNumber(left) - this.toNumber(right));
  },
};
