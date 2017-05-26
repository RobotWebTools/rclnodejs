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

const rclnodejs = require('bindings')('rclnodejs');

class Timer {
  constructor(handle, period, callback) {
    this._handle = handle;
    this._period = period;
    this.callback = callback;
  }

  get period() {
    return this._period;
  }

  get handle() {
    return this._handle;
  }

  isReady() {
    return rclnodejs.isTimerReady(this._handle);
  }

  isCanceled() {
    return rclnodejs.isTimerCanceled(this._handle);
  }

  cancel() {
    rclnodejs.cancelTimer(this._handle);
  }

  reset() {
    rclnodejs.resetTimer(this._handle);
  }

  timeSinceLastCall() {
    return rclnodejs.timerGetTimeSinceLastCall(this._handle);
  }

  timeUntilNextCall() {
    return rclnodejs.timerGetTimeUntilNextCall(this._handle);
  }
};

module.exports = Timer;
