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
const os = require('os');
const rclnodejs = require('../index.js');

describe('Rclnodejs createMessage() testing', function() {
  this.timeout(60 * 1000);

  before(function() {
    return rclnodejs.init();
  });

  after(function() {
    rclnodejs.shutdown();
  });

  it('Test .createMessage() for every message in system', function() {
    const packages = require('../rosidl_gen/packages.js');
    const installedPackagesRoot =
      os.type() === 'Windows_NT'
        ? process.env.AMENT_PREFIX_PATH.split(';')
        : process.env.AMENT_PREFIX_PATH.split(':');
    let promises = [];
    installedPackagesRoot.forEach(path => {
      let promise = packages.findPackagesInDirectory(path).then(pkgs => {
        pkgs.forEach(pkg => {
          pkg.messages.forEach(info => {
            const s =
              info.pkgName + '/' + info.subFolder + '/' + info.interfaceName;
            rclnodejs.createMessage(s);

            rclnodejs.createMessage({
              package: info.pkgName,
              type: info.subFolder,
              name: info.interfaceName,
            });
          });
        });
      });
      promises.push(promise);
    });
    return Promise.all(promises);
  });

  it('create by string', function() {
    const s = rclnodejs.createMessage('std_msgs/msg/String');

    assert('data' in s);
    assert(!s.data);
    s.data = 'this is fun';
    assert.equal(typeof s.data, 'string');
    assert.equal(s.data, 'this is fun');

    const t = s.constructor.classType.type();
    assert(t.pkgName === 'std_msgs');
    assert(t.subFolder === 'msg');
    assert(t.interfaceName === 'String');

    const classType = rclnodejs.require('std_msgs/msg/String');
    assert(s instanceof classType);
    assert(s.constructor.classType === classType);
  });

  it('create by object', function() {
    const s = rclnodejs.createMessage({
      package: 'std_msgs',
      type: 'msg',
      name: 'String',
    });

    assert('data' in s);
    assert(!s.data);
    s.data = 'this is fun';
    assert.equal(typeof s.data, 'string');
    assert.equal(s.data, 'this is fun');

    const t = s.constructor.classType.type();
    assert(t.pkgName === 'std_msgs');
    assert(t.subFolder === 'msg');
    assert(t.interfaceName === 'String');

    const classType = rclnodejs.require('std_msgs/msg/String');
    assert(s instanceof classType);
    assert(s.constructor.classType === classType);
  });

  [
    'std_msgs/msg/CString',
    'std_msgs/msg/String ',
    '/std_msgs/msg/String',
    ' std_msgs/msg/String',
    '/std_msgs/msg/CString',
    'std_msgs/String',
    '/std_msgs/String',
    'std_msgs/srv/String',
    'msg/String',
    '/msg/String',
    'unknown/msg/Unknown',
    { package: 'std_msgs', type: 'msg', name: 'CString' },
  ].forEach(testData => {
    it('expecting exception when passing ' + testData.toString(), function() {
      assert.throws(
        () => {
          rclnodejs.createMessage(testData);
        },
        function(e) {
          return e instanceof Error;
        }
      );
    });
  });
});

describe('Rclnodejs createMessageObject() testing', function() {
  this.timeout(60 * 1000);

  before(function() {
    return rclnodejs.init();
  });

  after(function() {
    rclnodejs.shutdown();
  });

  it('Test .createMessageObject() for every message in system', function() {
    const packages = require('../rosidl_gen/packages.js');
    const installedPackagesRoot =
      os.type() === 'Windows_NT'
        ? process.env.AMENT_PREFIX_PATH.split(';')
        : process.env.AMENT_PREFIX_PATH.split(':');
    let promises = [];
    installedPackagesRoot.forEach(path => {
      let promise = packages.findPackagesInDirectory(path).then(pkgs => {
        pkgs.forEach(pkg => {
          pkg.messages.forEach(info => {
            const s =
              info.pkgName + '/' + info.subFolder + '/' + info.interfaceName;
            rclnodejs.createMessageObject(s);

            rclnodejs.createMessageObject({
              package: info.pkgName,
              type: info.subFolder,
              name: info.interfaceName,
            });
          });
        });
      });
      promises.push(promise);
    });
    return Promise.all(promises);
  });

  it('create by string', function() {
    const s = rclnodejs.createMessageObject('std_msgs/msg/String');

    assert('data' in s);
    assert(!s.data);
    s.data = 'this is fun';
    assert.equal(typeof s.data, 'string');
    assert.equal(s.data, 'this is fun');
  });

  it('create by object', function() {
    const s = rclnodejs.createMessage({
      package: 'std_msgs',
      type: 'msg',
      name: 'String',
    });

    assert('data' in s);
    assert(!s.data);
    s.data = 'this is fun';
    assert.equal(typeof s.data, 'string');
    assert.equal(s.data, 'this is fun');
  });

  [
    'std_msgs/msg/CString',
    'std_msgs/msg/String ',
    '/std_msgs/msg/String',
    ' std_msgs/msg/String',
    '/std_msgs/msg/CString',
    'std_msgs/String',
    '/std_msgs/String',
    'std_msgs/srv/String',
    'msg/String',
    '/msg/String',
    'unknown/msg/Unknown',
    { package: 'std_msgs', type: 'msg', name: 'CString' },
  ].forEach(testData => {
    it('expecting exception when passing ' + testData.toString(), function() {
      assert.throws(
        () => {
          rclnodejs.createMessageObject(testData);
        },
        function(e) {
          return e instanceof Error;
        }
      );
    });
  });
});
