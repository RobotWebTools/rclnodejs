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
const path = require('path');

function sourceSetupScript(setupPath) {
  // Source the local_setup.sh to get environment variables
  const sourceResult = require('child_process').spawnSync(
    'bash',
    ['-c', `source ${setupPath} && env`],
    {
      encoding: 'utf8',
      timeout: 10000, // 10 second timeout
    }
  );

  if (sourceResult.error) {
    throw new Error(
      `Failed to source setup script: ${sourceResult.error.message}`
    );
  }

  if (sourceResult.status !== 0) {
    throw new Error(
      `Failed to source setup script with exit code ${sourceResult.status}`
    );
  }

  // Parse and apply environment variables to current process
  const envOutput = sourceResult.stdout;
  const envLines = envOutput.split('\n');

  envLines.forEach((line) => {
    const equalIndex = line.indexOf('=');
    if (equalIndex > 0) {
      const key = line.substring(0, equalIndex);
      const value = line.substring(equalIndex + 1);

      // Only update AMENT_PREFIX_PATH from the subprocess
      if (key === 'AMENT_PREFIX_PATH') {
        process.env[key] = value;
      }
    }
  });
}

function buildTestMessage() {
  // Build the custom_msg_test package synchronously before running the test
  const customMsgTestPath = path.join(__dirname, 'custom_msg_test');
  const buildResult = require('child_process').spawnSync('colcon', ['build'], {
    cwd: customMsgTestPath,
    stdio: 'inherit',
    timeout: 60000, // 60 second timeout
  });

  if (buildResult.error) {
    throw new Error(
      `Failed to build custom_msg_test package: ${buildResult.error.message}`
    );
  }

  if (buildResult.status !== 0) {
    throw new Error(`colcon build failed with exit code ${buildResult.status}`);
  }

  // Source the local_setup.sh to get environment variables
  const setupScriptPath = path.join(
    customMsgTestPath,
    'install',
    'local_setup.sh'
  );
  sourceSetupScript(setupScriptPath);
}

describe('ROSIDL Node.js message generator test suite', function () {
  this.timeout(60 * 1000);

  before(async function () {
    await rclnodejs.init();
  });

  after(function () {
    rclnodejs.shutdown();
  });

  it('Try require all message classes', function () {
    this.timeout(60 * 1000);
    const packages = require('../rosidl_gen/packages.js');
    const installedPackagesRoot =
      os.type() === 'Windows_NT'
        ? process.env.AMENT_PREFIX_PATH.split(';')
        : process.env.AMENT_PREFIX_PATH.split(':');
    let promises = [];
    installedPackagesRoot.forEach((path) => {
      let promise = packages.findPackagesInDirectory(path).then((pkgs) => {
        pkgs.forEach((pkg) => {
          pkg.messages.forEach((info) => {
            const s =
              info.pkgName + '/' + info.subFolder + '/' + info.interfaceName;
            assert(rclnodejs.require(s));
          });
        });
      });
      promises.push(promise);
    });
    return Promise.all(promises);
  });

  it('Try use std_msgs/msg/String.msg', function () {
    let String = rclnodejs.require('std_msgs').msg.String;
    let msg = new String();
    assert(!msg.data);
    msg.data = '123570'; // The only member of this message is .data (string)
    assert.equal(typeof msg.data, 'string');
    assert.equal(msg.data, '123570');

    for (let i = 0; i < 100; ++i) {
      msg.data = 'message + ' + i; // Testing string assignment multiple times (string de-allocation)
    }

    msg = new String();
    msg.data = '123570';
    assert.equal(msg.data, '123570');
  });

  it('Testing message with all-primitive members - ColorRGBA', function () {
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

  it('Testing copy-constructor - Duration', function () {
    let Duration = rclnodejs.require('builtin_interfaces').msg.Duration;
    let msg = new Duration();
    msg.sec = 1024;
    msg.nanosec = 0xaaaa5555;
    assert.equal(msg.sec, 1024);
    assert.equal(msg.nanosec, 0xaaaa5555);

    let msg2 = new Duration(msg);
    assert.equal(msg2.sec, 1024);
    assert.equal(msg2.nanosec, 0xaaaa5555);

    msg.sec = 2048;
    msg.nanosec = 0x5555aaaa;
    assert.equal(msg.sec, 2048);
    assert.equal(msg.nanosec, 0x5555aaaa);
    assert.equal(msg2.sec, 1024);
    assert.equal(msg2.nanosec, 0xaaaa5555);
  });

  it('Testing assignment of an all-primitive message - Time', function () {
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

  it('Testing a compound message - Pose', function () {
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

  it('Testing array - Int32', function () {
    let Int32 = rclnodejs.require('std_msgs').msg.Int32;
    let array = new Int32.ArrayType(5);

    assert(array.data instanceof Int32Array);
    assert(typeof array.data[5] === 'undefined'); // No such index
    assert.equal(array.size, 5);
    assert.equal(array.capacity, 5);

    // Assignment of message.data
    const int32Data = [153, 26, 777, 666, 999];
    for (let i = 0; i < int32Data.length; ++i) {
      array.data[i] = int32Data[i];
      assert.equal(array.data[i], int32Data[i]); // Verifying
    }

    // Array deep copy
    let array2 = new Int32.ArrayType();
    array2.copy(array);
    for (let i = 0; i < int32Data.length; ++i) {
      assert.equal(array2.data[i], int32Data[i]);
    }

    // Change array2
    for (let i = 0; i < array2.length; ++i) {
      array2.data[i] = 0;
    }

    // Values in array1 are NOT changed
    for (let i = 0; i < array.length; ++i) {
      assert.equal(array.data[i], int32Data[i]); // Verifying
    }

    // Resize
    array.size = 6;
    assert.equal(array.size, 6);
    assert.equal(array.capacity, 6);
  });

  it('Generate message at runtime', function () {
    const amentPrefixPathOriginal = process.env.AMENT_PREFIX_PATH;
    try {
      buildTestMessage();

      assert.doesNotThrow(() => {
        const Testing = rclnodejs.require('custom_msg_test/msg/Testing');
        const t = new Testing();
        assert.equal(typeof t, 'object');
        assert.equal(typeof t.x, 'number');
        assert.equal(typeof t.data, 'string');
      }, 'This function should not throw');
    } finally {
      process.env.AMENT_PREFIX_PATH = amentPrefixPathOriginal;
    }
  });

  it('Testing mrpt_msgs/msg/GraphSlamAgents from non-standard msg subfolder', function () {
    // GraphSlamAgents.msg lives under msg-common/ (non-standard subfolder name)
    // and references GraphSlamAgent.msg from msg-ros2/. This verifies that
    // packages with hyphenated subfolder names are generated and loadable.
    const GraphSlamAgents = rclnodejs.require('mrpt_msgs/msg/GraphSlamAgents');
    const GraphSlamAgent = rclnodejs.require('mrpt_msgs/msg/GraphSlamAgent');

    // Verify GraphSlamAgents can be instantiated with an empty list
    const agents = new GraphSlamAgents();
    assert.ok(agents.list);
    assert.equal(agents.list.size, 0);

    // Verify GraphSlamAgent fields and defaults
    const agent = new GraphSlamAgent();
    assert.equal(agent.name.data, '');
    assert.equal(agent.hostname.data, '');
    assert.equal(agent.ip_addr.data, '');
    assert.equal(agent.port, 0);
    assert.equal(agent.is_online.data, false);
    assert.equal(agent.last_seen_time.sec, 0);
    assert.equal(agent.last_seen_time.nanosec, 0);
    assert.equal(agent.topic_namespace.data, '');
    assert.equal(agent.agent_id, 0);

    // Verify field assignment
    agent.name.data = 'robot_1';
    agent.hostname.data = 'host1';
    agent.ip_addr.data = '192.168.1.17';
    agent.port = 11311;
    agent.is_online.data = true;
    agent.topic_namespace.data = 'robot_1';
    agent.agent_id = 17;

    assert.equal(agent.name.data, 'robot_1');
    assert.equal(agent.hostname.data, 'host1');
    assert.equal(agent.ip_addr.data, '192.168.1.17');
    assert.equal(agent.port, 11311);
    assert.equal(agent.is_online.data, true);
    assert.equal(agent.topic_namespace.data, 'robot_1');
    assert.equal(agent.agent_id, 17);
  });
});
