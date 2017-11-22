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

/* eslint-disable max-depth */
function deepEqual(msg, obj) {
  if (typeof obj !== 'object') {
    return msg === obj;
  }

  for (let i in obj) {
    const m = msg[i];
    const o = obj[i];

    if (typeof m === 'undefined') {
      return false;
    }

    const t = typeof o;
    if (t === 'string' || t === 'number' || t === 'boolean') {
      // A primitive value
      if (m !== o) {
        return false;
      }
    } else if (m.isROSArray) {
      // An array of message objects
      if (!Array.isArray(o)) return false;

      const data = m.data;
      for (let j in data) {
        if (!deepEqual(data[j], o[j])) {
          return false;
        }
      }
    } else {
      // A regular message object
      const equal = deepEqual(m, o);
      if (!equal) {
        return false;
      }
    }
  }

  return true;
}
/* eslint-enable max-depth */

module.exports = {
  deepEqual: deepEqual,
};
