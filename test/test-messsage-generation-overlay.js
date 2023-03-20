// Copyright (c) 2021 Wayne Parrott. All rights reserved.
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
const childProcess = require('child_process');
const fs = require('fs');
const path = require('path');
const rclnodejs = require('../index.js');
const generator = require('../rosidl_gen/index.js');

const GENERATED_PATH = path.join(__dirname, '..', 'generated');
const POINT_PATH = path.join(
  __dirname,
  '..',
  'generated',
  'geometry_msgs',
  'geometry_msgs__msg__Point.js'
);
const POSE_PATH = path.join(
  __dirname,
  '..',
  'generated',
  'geometry_msgs',
  'geometry_msgs__msg__Pose.js'
);

function clearRequireCache(key) {
  delete require.cache[key];
}

function generateMessages() {
  return generator.generateAll(true);
}

describe('override interface in overlay tests', function () {
  this.timeout(60 * 1000); // 90 seconds to run this test suite

  before(() => {
    // clear required cache of prior
    clearRequireCache(POINT_PATH);

    if (fs.existsSync(GENERATED_PATH)) {
      fs.rmSync(GENERATED_PATH, { recursive: true });
    }
  });

  after(async () => {
    if (fs.existsSync(path.join(__dirname, '..', 'generated'))) {
      fs.rmSync(path.join(__dirname, '..', 'generated'), { recursive: true });
    }

    clearRequireCache(POINT_PATH);
    clearRequireCache(POSE_PATH);

    await generateMessages();
  });

  it('overlay msg generation test', async () => {
    const amentPrefixPathOriginal = process.env.AMENT_PREFIX_PATH;
    assert.ok(amentPrefixPathOriginal, 'AMENT_PREFIX_PATH not found');

    const wsPath = path.join(
      __dirname,
      'overlay_test_ws',
      'install',
      'geometry_msgs'
    );
    let amentPrefixPath = wsPath + path.delimiter + amentPrefixPathOriginal;
    process.env.AMENT_PREFIX_PATH = amentPrefixPath;

    await generateMessages();

    const pointMsg = rclnodejs.createMessageObject('geometry_msgs/msg/Point');
    assert.ok(
      pointMsg.hasOwnProperty('data'),
      'Expected overridden Point msg to include "data" property'
    );

    // restore original AMENT_PREFIX_PATH envar setting
    process.env.AMENT_PREFIX_PATH = amentPrefixPathOriginal;
  });
});
