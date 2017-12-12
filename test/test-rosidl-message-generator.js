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
const rclnodejs = require('../index.js');

describe('ROSIDL Node.js message generator test suite', function() {
  before(function() {
    this.timeout(60 * 1000);
    return rclnodejs.init();
  });

  after(function() {
    rclnodejs.shutdown();
  });

  it('Try require all message classes', function() {
    this.timeout(60 * 1000);
    const packages = require('../rosidl_gen/packages.js');
    const installedPackagesRoot = process.env.AMENT_PREFIX_PATH.split(':');
    let promises = [];
    installedPackagesRoot.forEach((path) => {
      let promise = packages.findPackagesInDirectory(path).then((pkgs) => {
        pkgs.forEach((pkg) => {
          pkg.messages.forEach((info) => {
            if (info.subFolder === 'msg') {
              assert(rclnodejs.require(info.pkgName).msg[info.interfaceName]);
            }
          });
        });
      });
      promises.push(promise);
    });
    return Promise.all(promises);
  });

  it('Try use std_msgs/msg/String.msg', function() {
    let String = rclnodejs.require('std_msgs').msg.String;
    let msg = new String();
    assert(!msg.data);
    msg.data = '123570'; // The only member of this message is .data (string)
    assert.equal(typeof msg.data, 'string');
    assert.equal(msg.data, '123570');

    for (let i = 0; i < 100; ++ i) {
      msg.data = 'message + ' + i;  // Testing string assignment multiple times (string de-allocation)
    }

    msg = new String();
    msg.data = '123570';
    assert.equal(msg.data, '123570');
  });

  it('Testing message with all-primitive members - ColorRGBA', function() {
    let ColorRGBA = rclnodejs.require('std_msgs').msg.ColorRGBA;
    let msg = new ColorRGBA();
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
    let Duration = rclnodejs.require('builtin_interfaces').msg.Duration;
    let msg = new Duration();
    msg.sec = 1024;
    msg.nanosec = 0xAAAA5555;
    assert.equal(msg.sec, 1024);
    assert.equal(msg.nanosec, 0xAAAA5555);

    let msg2 = new Duration(msg);
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
    let Time = rclnodejs.require('builtin_interfaces').msg.Time;
    let msg = new Time();
    msg.sec = 120;
    msg.nanosec = 777;

    assert.equal(msg.sec, 120);
    assert.equal(msg.nanosec, 777);

    let msg2 = new Time();
    msg2.copy(msg);

    msg2.sec = 240;
    msg2.nanosec = 888;
    assert.equal(msg2.sec, 240);
    assert.equal(msg2.nanosec, 888);

    assert.equal(msg.sec, 120);
    assert.equal(msg.nanosec, 777);
  });

  it('Testing a compound message - Pose', function() {
    let Pose = rclnodejs.require('geometry_msgs').msg.Pose;
    let Point = rclnodejs.require('geometry_msgs').msg.Point;
    let Quaternion = rclnodejs.require('geometry_msgs').msg.Quaternion;
    let msg = new Pose();
    assert(msg.position instanceof Point);
    assert(msg.orientation instanceof Quaternion);

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
    let copy = new Pose(msg);
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

  it('Testing constants - GoalStatus', function() {
    let GoalStatus = rclnodejs.require('actionlib_msgs').msg.GoalStatus;
    let msg = new GoalStatus();
    assert.equal(typeof msg.PENDING, 'undefined');
    assert.equal(typeof GoalStatus.PENDING, 'number');
    assert.equal(GoalStatus.PENDING, 0);

    /*
    uint8 PENDING         = 0   # The goal has yet to be processed by the action server.
    uint8 ACTIVE          = 1   # The goal is currently being processed by the action server.
    uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
                                #   and has since completed its execution (Terminal State).
    uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server
                                #   (Terminal State).
    uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
                                #    to some failure (Terminal State).
    uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
                                #    because the goal was unattainable or invalid (Terminal State).
    uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
                                #    and has not yet completed execution.
    uint8 RECALLING       = 7   # The goal received a cancel request before it started executing, but
                                #    the action server has not yet confirmed that the goal is canceled.
    uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
                                #    and was successfully cancelled (Terminal State).
    uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not
    */

    const constantsName  = ['PENDING', 'ACTIVE', 'PREEMPTED', 'SUCCEEDED', 'ABORTED',
      'REJECTED', 'PREEMPTING', 'RECALLING', 'RECALLED', 'LOST'];
    const cconstantsValue = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9];
    for (let i = 0; i < constantsName.length; ++ i) {
      assert.equal(typeof msg[constantsName[i]], 'undefined');
      assert.equal(typeof GoalStatus[constantsName[i]], 'number');
      assert.equal(GoalStatus[constantsName[i]], cconstantsValue[i]);
    }
  });

  it('Testing array - Int32', function() {
    let Int32 = rclnodejs.require('std_msgs').msg.Int32;
    let array = new Int32.ArrayType(5);

    assert(array.data instanceof Int32Array);
    assert(typeof array.data[5] === 'undefined');  // No such index
    assert.equal(array.size, 5);
    assert.equal(array.capacity, 5);

    // Assignment of message.data
    const int32Data = [153, 26, 777, 666, 999];
    for (let i = 0; i < int32Data.length; ++ i) {
      array.data[i] = int32Data[i];
      assert.equal(array.data[i], int32Data[i]);  // Verifying
    }

    // Array deep copy
    let array2 = new Int32.ArrayType();
    array2.copy(array);
    for (let i = 0; i < int32Data.length; ++ i) {
      assert.equal(array2.data[i], int32Data[i]);
    }

    // Change array2
    for (let i = 0; i < array2.length; ++ i) {
      array2.data[i] = 0;
    }

    // Values in array1 are NOT changed
    for (let i = 0; i < array.length; ++ i) {
      assert.equal(array.data[i], int32Data[i]);  // Verifying
    }

    // Resize
    array.size = 6;
    assert.equal(array.size, 6);
    assert.equal(array.capacity, 6);
  });
});
