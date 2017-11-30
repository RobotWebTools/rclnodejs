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

describe('Rclnodejs createMessageObject() testing', function() {
  this.timeout(60 * 1000);

  before(function() {
    return rclnodejs.init();
  });

  after(function() {
    rclnodejs.shutdown();
  });

  it('create by string', function() {
    const s = rclnodejs.createMessageObject('std_msgs/msg/String');

    assert('data' in s);
    assert(!s.data);
    s.data = 'this is fun';
    assert.equal(typeof s.data, 'string');
    assert.equal(s.data, 'this is fun');

    const t = s.classType.type();
    assert(t.pkgName === 'std_msgs');
    assert(t.subFolder === 'msg');
    assert(t.interfaceName === 'String');

    const classType = rclnodejs.require('std_msgs/msg/String');
    assert(s instanceof classType);
    assert(s.classType === classType);
  });

  it('create by object', function() {
    const s = rclnodejs.createMessageObject({
      package: 'std_msgs',
      type: 'msg',
      name: 'String',
    });

    assert('data' in s);
    assert(!s.data);
    s.data = 'this is fun';
    assert.equal(typeof s.data, 'string');
    assert.equal(s.data, 'this is fun');

    const t = s.classType.type();
    assert(t.pkgName === 'std_msgs');
    assert(t.subFolder === 'msg');
    assert(t.interfaceName === 'String');

    const classType = rclnodejs.require('std_msgs/msg/String');
    assert(s instanceof classType);
    assert(s.classType === classType);
  });

});
