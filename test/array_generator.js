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
  const length = Math.floor(Math.random() * (maxLength -1) + 1);
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

function positive(v) {return v;}
function negative(v) {return -v;}
function noRound(v) {return v;}

// const arrayGen = require('./array_generator.js');
// const arrayLength = 100 * 1024;
// arrayGen.generateValues(Float32Array, arrayLength, 100000000, arrayGen.negative, arrayGen.noRound);
// arrayGen.generateValues(Float32Array, arrayLength, 10000, arrayGen.negative, arrayGen.noRound);
// arrayGen.generateValues(Float64Array, arrayLength, Number.MAX_VALUE, arrayGen.negative, arrayGen.noRound);
// arrayGen.generateValues(Float64Array, arrayLength, 10000, arrayGen.negative, arrayGen.noRound);
// arrayGen.generateValues(Int8Array,    arrayLength, 128, arrayGen.negative, Math.floor);
// arrayGen.generateValues(Int16Array,   arrayLength, 32768, arrayGen.negative, Math.floor);
// arrayGen.generateValues(Int32Array,   arrayLength, 2147483648, arrayGen.negative, Math.floor);
// arrayGen.generateValues(Uint8Array,   arrayLength, 256, arrayGen.positive, Math.floor);
// arrayGen.generateValues(Uint16Array,  arrayLength, 65536, arrayGen.positive, Math.floor);
// arrayGen.generateValues(Uint32Array,  arrayLength, 4294967296, arrayGen.positive, Math.floor);
// arrayGen.generateValues(Array, arrayLength, 256, arrayGen.positive, Math.floor);

module.exports = {
  generateValues: generateValues,
  positive: positive,
  negative: negative,
  noRound: noRound,
};
