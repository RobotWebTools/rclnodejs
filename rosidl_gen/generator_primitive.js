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

const StructType = require('ref-struct');
const ref = require('ref');
const rclnodejs = require('bindings')('rclnodejs');

/* eslint-disable camelcase */

const rosidl_generator_nodejs__String = StructType({
  data: ref.types.CString,
  size: ref.types.size_t,
  capacity: ref.types.size_t,
});

function bindingsStringInit(buf) {
  if (! buf instanceof Buffer) {
    throw new TypeError('Invalid argument: should provide a Node Buffer to bindingsStringInit()');
  }

  return rclnodejs.rosIDLStringInit(buf);
}

function bindingsStringAssign(buf, text) {
  if (! buf instanceof Buffer) {
    throw new TypeError('Invalid argument: should provide a Node Buffer to bindingsStringInit()');
  }

  let strPtr = new Buffer(text + '\0');
  strPtr.type = ref.types.CString;
  return rclnodejs.rosIDLStringAssign(buf, strPtr);
}

module.exports = {
  bool: ref.types.bool,
  int8: ref.types.int8,
  uint8: ref.types.uint8,
  int16: ref.types.int16,
  uint16: ref.types.uint16,
  int32: ref.types.int32,
  uint32: ref.types.uint32,
  int64: ref.types.int64,
  uint64: ref.types.uint64,
  float64: ref.types.double,
  float32: ref.types.float,
  char: ref.types.char,
  byte: ref.types.byte,

  string: rosidl_generator_nodejs__String,
  stringInit: bindingsStringInit,
  stringAssignFunc: bindingsStringAssign,
};

/* eslint-enable camelcase */
