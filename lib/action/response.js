// Copyright (c) 2020 Matt Richard. All rights reserved.
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

/**
 * Goal events that cause state transitions.
 * @ignore
 * @readonly
 * @enum {number}
 */
const GoalEvent = {
  /** @member {number} */
  EXECUTE: 0,
  /** @member {number} */
  CANCEL_GOAL: 1,
  /** @member {number} */
  SUCCEED: 2,
  /** @member {number} */
  ABORT: 3,
  /** @member {number} */
  CANCELED: 4,
};

/**
 * Possible goal responses.
 * @readonly
 * @enum {number}
 */
const GoalResponse = {
  /** @member {number} */
  REJECT: 1,
  /** @member {number} */
  ACCEPT: 2,
};

/**
 * Possible cancel responses.
 * @readonly
 * @enum {number}
 */
const CancelResponse = {
  /** @member {number} */
  REJECT: 1,
  /** @member {number} */
  ACCEPT: 2,
};

module.exports = {
  GoalEvent,
  GoalResponse,
  CancelResponse,
};
