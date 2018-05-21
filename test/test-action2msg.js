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
const fs = require('fs');
const path = require('path');
const rclnodejs = require('../index.js');

describe('Testing message files generated from an action file', function() {
  let testRootDir = __dirname;
  let testActionDir = path.join(testRootDir, 'dodishes_action');
  let pkgRootDir = path.dirname(testRootDir);
  let msgGenRootDir = path.join(pkgRootDir, 'generated');

  this.timeout(60 * 1000);

  before(function() {
    return rclnodejs.init();    
  });

  after(function() {
    rclnodejs.shutdown();
  });


  describe('Message files generated from an action file', function() {
    let actionName = 'DoDishes';
    let testCasesData = [
      'ActionFeedback.msg',
      'ActionGoal.msg',
      'Action.msg',
      'ActionResult.msg',
      'Feedback.msg',
      'Goal.msg',
      'Result.msg'
    ];

    testCasesData.forEach((testData) => {
      it(testData + ' should be generated', function() {
        // eslint-disable-next-line
        assert.ok(fs.existsSync(path.join(testActionDir, actionName + testData)));
      });
    });
  });

  describe('JavaScript message files generated', function() {
    let pkgPrefix = 'dodishes_action';
    let exampleName = 'DoDishes';
    let testCasesData = [
      'ActionFeedback',
      'ActionGoal',
      'Action',
      'ActionResult',
      'Feedback',
      'Goal',
      'Result'
    ];

    testCasesData.forEach((testData) => {
      let pkgName = pkgPrefix + '/msg/' + exampleName + testData;
      it(pkgName + ' can be required', function() {
        rclnodejs.require(pkgName);
      });
    });
  });
});

