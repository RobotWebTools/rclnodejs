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

const assert = require('assert');
const {message} = require('../index.js');

describe('ROSIDL Node.js message generator test suite', function(){
  it('Try generate all messages', function() {
    return new Promise(function(resolve, reject) {
      message.generateAll().then((msgTypeList) => {
        assert(msgTypeList.length > 0);

        msgTypeList.forEach((msgType) => {
          try {
            const MessageClass = message.getMessageClass(msgType);
          } catch (e) {
            reject(e);
          }
        });
        resolve();
      });
    }); // new Promise
  });
});
