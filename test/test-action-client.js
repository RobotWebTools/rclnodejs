// Copyright (c) 2020 Matt Richard. All rights reserved.
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
const sinon = require('sinon');
const { v4: uuidv4 } = require('uuid');
const assertUtils = require('./utils.js');
const rclnodejs = require('../index.js');

describe('rclnodejs action client', function () {
  let node;
  let server;
  this.timeout(60 * 1000);
  let fibonacci = 'rclnodejs_test_msgs/action/Fibonacci';
  let Fibonacci;

  let publishFeedback = null;

  function createUuid() {
    let uuid = uuidv4().replace(/-/g, '');
    let bytes = Uint8Array.from(Buffer.from(uuid, 'hex'));

    let UUID = rclnodejs.require('unique_identifier_msgs/msg/UUID');
    let goalUuid = new UUID();
    goalUuid.uuid = bytes;

    return goalUuid;
  }

  function uuidAsString(uuid) {
    return [].slice.call(uuid).join(',');
  }

  async function executeCallback(goalHandle) {
    if (
      publishFeedback &&
      uuidAsString(publishFeedback.uuid) ===
        uuidAsString(goalHandle.goalId.uuid)
    ) {
      goalHandle.publishFeedback(new Fibonacci.Feedback());
    }

    // Delay slightly to give tests time to cancel if needed
    await assertUtils.createDelay(50);

    if (goalHandle.isCancelRequested) {
      goalHandle.canceled();
    } else {
      goalHandle.succeed();
    }

    return new Fibonacci.Result();
  }

  before(function () {
    return rclnodejs.init();
  });

  after(function () {
    rclnodejs.shutdown();
  });

  beforeEach(function () {
    Fibonacci = rclnodejs.require(fibonacci);

    node = rclnodejs.createNode('action_client_node');
    server = new rclnodejs.ActionServer(
      node,
      fibonacci,
      'fibonacci',
      executeCallback,
      null,
      null,
      () => rclnodejs.CancelResponse.ACCEPT
    );

    rclnodejs.spin(node);
  });

  afterEach(function () {
    publishFeedback = null;
    server.destroy();
    node.destroy();
  });

  it('Test defaults', function () {
    let client = new rclnodejs.ActionClient(node, fibonacci, 'fibonacci');

    assert.ok(server.options);
    assert.ok(client.qos);
    assert.strictEqual(
      client.qos.goalServiceQosProfile,
      rclnodejs.QoS.profileServicesDefault
    );
    assert.strictEqual(
      client.qos.resultServiceQosProfile,
      rclnodejs.QoS.profileServicesDefault
    );
    assert.strictEqual(
      client.qos.cancelServiceQosProfile,
      rclnodejs.QoS.profileServicesDefault
    );
    assert.deepStrictEqual(
      client.qos.feedbackSubQosProfile,
      new rclnodejs.QoS(
        rclnodejs.QoS.HistoryPolicy.RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT,
        10
      )
    );
    assert.strictEqual(
      client.qos.statusSubQosProfile,
      rclnodejs.QoS.profileActionStatusDefault
    );

    client.destroy();
  });

  it('Test no wait for server', async function () {
    let client = new rclnodejs.ActionClient(node, fibonacci, 'not_fibonacci');

    let result = await client.waitForServer(0);

    assert.strictEqual(result, false);

    client.destroy();
  });

  it('Test wait for server timeout', async function () {
    let client = new rclnodejs.ActionClient(node, fibonacci, 'not_fibonacci');

    let result = await client.waitForServer(1000);

    assert.strictEqual(result, false);

    client.destroy();
  });

  it('Test wait for server', async function () {
    let client = new rclnodejs.ActionClient(node, fibonacci, 'fibonacci');

    let result = await client.waitForServer(2000);

    assert.ok(result);

    client.destroy();
  });

  it('Test send goal', async function () {
    let client = new rclnodejs.ActionClient(node, fibonacci, 'fibonacci');

    let result = await client.waitForServer(2000);
    assert.ok(result);

    let goalHandle = await client.sendGoal(new Fibonacci.Goal());
    assert.ok(goalHandle.isAccepted());

    result = await goalHandle.getResult();
    assert.ok(result);

    client.destroy();
  });

  it('Test send goal with feedback', async function () {
    let client = new rclnodejs.ActionClient(node, fibonacci, 'fibonacci');

    let feedbackCallback = sinon.spy(function (feedback) {
      assert.ok(feedback);
    });

    let goalUuid = createUuid();

    publishFeedback = goalUuid;

    let result = await client.waitForServer(2000);
    assert.ok(result);

    let goalHandle = await client.sendGoal(
      new Fibonacci.Goal(),
      feedbackCallback,
      goalUuid
    );
    assert.ok(goalHandle.isAccepted());

    await goalHandle.getResult();
    assert.ok(goalHandle.isSucceeded());
    assert.ok(result);

    assert.ok(feedbackCallback.calledOnce);

    client.destroy();
  });

  it('Test send goal with feedback for another goal', async function () {
    let client = new rclnodejs.ActionClient(node, fibonacci, 'fibonacci');

    let goal1Uuid = createUuid();
    let goal2Uuid = createUuid();

    let feedback1Callback = sinon.spy();
    let feedback2Callback = sinon.spy();

    // Only publish feedback for the first goal
    publishFeedback = goal1Uuid;

    let result = await client.waitForServer(2000);
    assert.ok(result);

    const [goal1Handle, goal2Handle] = await Promise.all([
      client.sendGoal(new Fibonacci.Goal(), feedback1Callback, goal1Uuid),
      client.sendGoal(new Fibonacci.Goal(), feedback2Callback, goal2Uuid),
    ]);

    await goal1Handle.getResult();
    await goal2Handle.getResult();

    assert.ok(feedback1Callback.calledOnce);
    assert.ok(feedback2Callback.notCalled);

    client.destroy();
  });

  it('Test send goal multiple', async function () {
    let client = new rclnodejs.ActionClient(node, fibonacci, 'fibonacci');

    let result = await client.waitForServer(2000);
    assert.ok(result);

    const [goal1Handle, goal2Handle, goal3Handle] = await Promise.all([
      client.sendGoal(new Fibonacci.Goal()),
      client.sendGoal(new Fibonacci.Goal()),
      client.sendGoal(new Fibonacci.Goal()),
    ]);

    assert.ok(goal1Handle.accepted);
    assert.ok(goal2Handle.accepted);
    assert.ok(goal3Handle.accepted);

    const [result1, result2, result3] = await Promise.all([
      goal1Handle.getResult(),
      goal2Handle.getResult(),
      goal3Handle.getResult(),
    ]);

    assert.ok(result1);
    assert.ok(result2);
    assert.ok(result3);

    client.destroy();
  });

  it('Test send goal with no server', async function () {
    let client = new rclnodejs.ActionClient(node, fibonacci, 'not_fibonacci');

    try {
      let goalHandlePromise = client.sendGoal(new Fibonacci.Goal());

      // Spy on the promise to see if it's ever resolved
      let spy = sinon.spy(goalHandlePromise, 'then');

      // Spin for 2 seconds
      await assertUtils.createDelay(2000);

      assert.ok(spy.notCalled);
    } finally {
      client.destroy();
    }
  });

  it('Test send cancel', async function () {
    let client = new rclnodejs.ActionClient(node, fibonacci, 'fibonacci');

    let result = await client.waitForServer(2000);
    assert.ok(result);

    let goalHandle = await client.sendGoal(new Fibonacci.Goal());
    assert.ok(goalHandle.isAccepted());

    result = await goalHandle.cancelGoal();
    assert.ok(result);

    assert.strictEqual(
      uuidAsString(result.goals_canceling[0].goal_id.uuid),
      uuidAsString(goalHandle.goalId.uuid)
    );

    client.destroy();
  });
});
