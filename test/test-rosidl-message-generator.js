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

  it('Testing message with all-primitive members - ColorRGBA', function() {
    const MessageClass = message.getMessageClass('std_msgs', 'msg', 'ColorRGBA');
    assert.equal(MessageClass.name, 'std_msgs__msg__ColorRGBA');
    let msg = new MessageClass();
    msg.r = 0.5;
    msg.g = 0.25;
    msg.b = 0.125;
    msg.a = 0.75;
    assert.equal(msg.r, 0.5);
    assert.equal(msg.g, 0.25);
    assert.equal(msg.b, 0.125);
    assert.equal(msg.a, 0.75);
  });

  it('Testing copy-constructor - Duration', function() {
    const MessageClass = message.getMessageClass('builtin_interfaces', 'msg', 'Duration');
    assert.equal(MessageClass.name, 'builtin_interfaces__msg__Duration');
    let msg = new MessageClass();
    msg.sec = 1024;
    msg.nanosec = 0xAAAA5555;
    assert.equal(msg.sec, 1024);
    assert.equal(msg.nanosec, 0xAAAA5555);

    let msg2 = new MessageClass(msg);
    assert.equal(msg2.sec, 1024);
    assert.equal(msg2.nanosec, 0xAAAA5555);

    msg.sec = 2048;
    msg.nanosec = 0x5555AAAA;
    assert.equal(msg.sec, 2048);
    assert.equal(msg.nanosec, 0x5555AAAA);
    assert.equal(msg2.sec, 1024);
    assert.equal(msg2.nanosec, 0xAAAA5555);
  });

  it('Testing assignment of an all-primitive message - Time', function() {
    const MessageClass = message.getMessageClass('builtin_interfaces', 'msg', 'Time');
    assert.equal(MessageClass.name, 'builtin_interfaces__msg__Time');
    let msg = new MessageClass();
    msg.sec = 120;
    msg.nanosec = 777;

    assert.equal(msg.sec, 120);
    assert.equal(msg.nanosec, 777);

    let msg2 = new MessageClass();
    msg2.copy(msg);

    msg2.sec = 240;
    msg2.nanosec = 888;
    assert.equal(msg2.sec, 240);
    assert.equal(msg2.nanosec, 888);

    assert.equal(msg.sec, 120);
    assert.equal(msg.nanosec, 777);
  });

  it('Testing a compound message - Pose', function() {
    const MessageClass = message.getMessageClass('geometry_msgs', 'msg', 'Pose');
    const geometry_msgs__msg__Point = message.getMessageClass('geometry_msgs', 'msg', 'Point');
    const geometry_msgs__msg__Quaternion = message.getMessageClass('geometry_msgs', 'msg', 'Quaternion');

    assert.equal(MessageClass.name, 'geometry_msgs__msg__Pose');
    let msg = new MessageClass();
    assert(msg.position instanceof geometry_msgs__msg__Point);
    assert(msg.orientation instanceof geometry_msgs__msg__Quaternion);

    // Setter + getter
    msg.position.x = 123.5;
    msg.position.y = 456.25;
    msg.position.z = 789.125;
    msg.orientation.x = 1234.125;
    msg.orientation.y = 4567.25;
    msg.orientation.z = 7890.5;
    assert.equal(msg.position.x, 123.5);
    assert.equal(msg.position.y, 456.25);
    assert.equal(msg.position.z, 789.125);
    assert.equal(msg.orientation.x, 1234.125);
    assert.equal(msg.orientation.y, 4567.25);
    assert.equal(msg.orientation.z, 7890.5);


    // Copy ctor
    let copy = new MessageClass(msg);
    assert.equal(copy.position.x, 123.5);
    assert.equal(copy.position.y, 456.25);
    assert.equal(copy.position.z, 789.125);
    assert.equal(copy.orientation.x, 1234.125);
    assert.equal(copy.orientation.y, 4567.25);
    assert.equal(copy.orientation.z, 7890.5);

    // Does not interfere -- 1
    msg.position.x = 1230.5;
    msg.position.y = 4560.25;
    msg.position.z = 7890.125;
    assert.equal(copy.position.x, 123.5);
    assert.equal(copy.position.y, 456.25);
    assert.equal(copy.position.z, 789.125);
    assert.equal(msg.position.x, 1230.5);
    assert.equal(msg.position.y, 4560.25);
    assert.equal(msg.position.z, 7890.125);
    assert.equal(msg.orientation.x, 1234.125);
    assert.equal(msg.orientation.y, 4567.25);
    assert.equal(msg.orientation.z, 7890.5);

    // Does not interfere -- 2
    copy.position.x = 12301.5;
    copy.position.y = 45601.25;
    copy.position.z = 78901.125;
    assert.equal(msg.position.x, 1230.5);
    assert.equal(msg.position.y, 4560.25);
    assert.equal(msg.position.z, 7890.125);
    assert.equal(msg.orientation.x, 1234.125);
    assert.equal(msg.orientation.y, 4567.25);
    assert.equal(msg.orientation.z, 7890.5);
    assert.equal(copy.position.x, 12301.5);
    assert.equal(copy.position.y, 45601.25);
    assert.equal(copy.position.z, 78901.125);
  });

});
