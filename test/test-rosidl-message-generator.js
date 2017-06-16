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
    this.timeout(100 * 1000);
    return new Promise(function(resolve, reject) {
      message.generateAll().then((msgTypeList) => {
        const DEFAULT_NUMBER_OF_MESSAGES = 127; // # of a new standard ROS 2.0 build
        assert(msgTypeList.length >= DEFAULT_NUMBER_OF_MESSAGES);

        // Try require all message class
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

  it('Try use stdmsgs/msg/String.msg', function() {
    const MessageClass1 = message.getMessageClass(message.getMessageType('std_msgs', 'msg', 'String'));
    assert.equal(MessageClass1.name, 'std_msgs__msg__String');
    let msg = new MessageClass1();
    assert(!msg.data);
    msg.data = '123570'; // The only member of this message is .data (string)
    assert.equal(typeof msg.data, 'string');
    assert.equal(msg.data, '123570');

    const MessageClass2 = message.getMessageClass('std_msgs', 'msg', 'String'); // override func
    assert.equal(MessageClass2.name, 'std_msgs__msg__String');
    msg = new MessageClass2();
    msg.data = '123570';
    assert.equal(msg.data, '123570');
  });

});
