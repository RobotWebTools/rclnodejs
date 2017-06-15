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

/* eslint-disable camelcase */

const rosidl_generator_c__String = StructType({
  data: ref.types.CString,
  size: ref.types.size_t,
  capacity: ref.types.size_t,
});

function rosidl_generator_c__String__assign(str, text) {
  let strBuf = new Buffer(text + '\0');
  strBuf.type = ref.types.CString;
  str.data = strBuf;
  str.size = text.length;
  str.capacity = text.length + 1;
}

module.exports = {
  rosidl_generator_c__String: rosidl_generator_c__String,
  rosidl_generator_c__String__assign: rosidl_generator_c__String__assign,
};

/* eslint-enable camelcase */
