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
const assertUtils = require('./utils.js');
const assertThrowsError = assertUtils.assertThrowsError;

const NODE_NAME = 'lifecycle_node';

// let StateInterface = rclnodejs.createMessage('lifecycle_msgs/msg/State').constructor;
// let TransitionInterface = rclnodejs.createMessage('lifecycle_msgs/msg/Transition').constructor;

describe('LifecyclePublisher test suite', function () {
  let node;
  this.timeout(60 * 1000);

  before(async function () {
    await rclnodejs.init();
  });

  beforeEach(function () {
    node = rclnodejs.createLifecycleNode(NODE_NAME);
    rclnodejs.spin(node);
  });

  afterEach(function () {
    node.destroy();
  });

  after(function () {
    rclnodejs.shutdown();
  });

  it('LifecyclePublisher create', function () {
    const publisher = node.createPublisher('std_msgs/msg/String', 'test');
    assert.ok(publisher);

    const lifecyclePublisher = node.createLifecyclePublisher(
      'std_msgs/msg/String',
      'test'
    );
    assert.ok(lifecyclePublisher);
  });

  it('LifecyclePublisher activate/deactivate', function () {
    const lifecyclePublisher = node.createLifecyclePublisher(
      'std_msgs/msg/String',
      'test'
    );
    assert.ok(lifecyclePublisher);

    assert.ok(!lifecyclePublisher.isActivated());

    lifecyclePublisher.activate();
    assert.ok(lifecyclePublisher.isActivated());

    lifecyclePublisher.deactivate();
    assert.ok(!lifecyclePublisher.isActivated());

    lifecyclePublisher.onActivate();
    assert.ok(lifecyclePublisher.isActivated());

    lifecyclePublisher.onDeactivate();
    assert.ok(!lifecyclePublisher.isActivated());
  });

  it('LifecyclePublisher publish', async function () {
    const lifecyclePublisher = node.createLifecyclePublisher(
      'std_msgs/msg/String',
      'test'
    );
    assert.ok(lifecyclePublisher);

    const TEST_MSG = 'test-msg';
    const waitTime = 1000; // millis
    let cbCnt = 0;
    let subscriber = node.createSubscription(
      'std_msgs/msg/String',
      'test',
      (msg) => {
        cbCnt++;
        assert.equal(msg.data, TEST_MSG);
      }
    );

    lifecyclePublisher.publish(TEST_MSG);
    await assertUtils.createDelay(waitTime);
    assert.strictEqual(cbCnt, 0);

    lifecyclePublisher.activate();
    lifecyclePublisher.publish(TEST_MSG);
    await assertUtils.createDelay(waitTime);
    assert.strictEqual(cbCnt, 1);

    lifecyclePublisher.deactivate();
    lifecyclePublisher.publish(TEST_MSG);
    await assertUtils.createDelay(waitTime);
    assert.strictEqual(cbCnt, 1);

    lifecyclePublisher.activate();
    lifecyclePublisher.publish(TEST_MSG);
    await assertUtils.createDelay(waitTime);
    assert.strictEqual(cbCnt, 2);
  });
});
