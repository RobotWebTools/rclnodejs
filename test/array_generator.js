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

function generateValues(Type, maxLength, range, negative, round, extra) {
  if (!extra) extra = [];
  const length = Math.floor(Math.random() * (maxLength - 1) + 1);
  let array = new Type(length + extra.length);
  for (let i = 0; i < length; ++i) {
    let value = round(Math.random() * range);
    if (Math.random() < 0.5) {
      value = negative(value);
    }
    array[i] = value;
  }
  for (let i = length; i < length + extra.length; ++i) {
    array[i] = extra[i - length];
  }
  return array;
}

function positive(v) {
  return v;
}
function negative(v) {
  return -v;
}
function noRound(v) {
  return v;
}

module.exports = {
  generateValues: generateValues,
  positive: positive,
  negative: negative,
  noRound: noRound,
};
