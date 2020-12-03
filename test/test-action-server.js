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
const deepEqual = require('deep-equal');
const { v4: uuidv4 } = require('uuid');
const assertUtils = require('./utils.js');
const rclnodejs = require('../index.js');

describe('rclnodejs action server', function () {
  let node;
  let client;
  this.timeout(60 * 1000);
  let fibonacci = 'rclnodejs_test_msgs/action/Fibonacci';
  let Fibonacci;
  let GoalStatus;

  function executeCallback(goalHandle) {
    goalHandle.succeed();
    return new Fibonacci.Result();
  }

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

  before(function () {
    return rclnodejs.init();
  });

  after(function () {
    rclnodejs.shutdown();
  });

  beforeEach(function () {
    Fibonacci = rclnodejs.require(fibonacci);
    GoalStatus = rclnodejs.require('action_msgs/msg/GoalStatus');

    node = rclnodejs.createNode('action_server_node');
    client = new rclnodejs.ActionClient(node, fibonacci, 'fibonacci');

    rclnodejs.spin(node);
  });

  afterEach(function () {
    client.destroy();
    node.destroy();
  });

  it('Test constructor defaults', function () {
    let server = new rclnodejs.ActionServer(
      node,
      fibonacci,
      'fibonacci',
      executeCallback
    );

    assert.ok(server.options);
    assert.strictEqual(server.options.resultTimeout, 900);
    assert.ok(server.qos);
    assert.strictEqual(
      server.qos.goalServiceQosProfile,
      rclnodejs.QoS.profileServicesDefault
    );
    assert.strictEqual(
      server.qos.resultServiceQosProfile,
      rclnodejs.QoS.profileServicesDefault
    );
    assert.strictEqual(
      server.qos.cancelServiceQosProfile,
      rclnodejs.QoS.profileServicesDefault
    );
    assert.deepStrictEqual(
      server.qos.feedbackSubQosProfile,
      new rclnodejs.QoS(
        rclnodejs.QoS.HistoryPolicy.RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT,
        10
      )
    );
    assert.strictEqual(
      server.qos.statusSubQosProfile,
      rclnodejs.QoS.profileActionStatusDefault
    );

    server.destroy();
  });

  it('Test constructor no defaults', function () {
    let server = new rclnodejs.ActionServer(
      node,
      fibonacci,
      'fibonacci',
      executeCallback,
      () => rclnodejs.GoalResponse.ACCEPT,
      null,
      () => rclnodejs.CancelResponse.REJECT,
      { resultTimout: 300 }
    );

    server.destroy();
  });

  it('Test single goal accept', async function () {
    let goalUuid = createUuid();
    let goalOrder = 10;

    function goalCallback(goal) {
      assert.strictEqual(goal.order, goalOrder);
      return rclnodejs.GoalResponse.ACCEPT;
    }

    function handleAcceptedCallback(goalHandle) {
      assert.strictEqual(goalHandle.status, GoalStatus.STATUS_ACCEPTED);
      assert.strictEqual(
        uuidAsString(goalHandle.goalId.uuid),
        uuidAsString(goalUuid.uuid)
      );
      assert.strictEqual(goalHandle.request.order, goalOrder);

      goalHandle.execute();
    }

    let server = new rclnodejs.ActionServer(
      node,
      fibonacci,
      'fibonacci',
      executeCallback,
      goalCallback,
      handleAcceptedCallback
    );

    let goal = new Fibonacci.Goal();
    goal.order = goalOrder;

    await client.waitForServer(1000);
    const handle = await client.sendGoal(goal, null, goalUuid);
    assert.ok(handle.accepted);

    let result = await handle.getResult();
    assert.ok(result);

    server.destroy();
  });

  it('Test single goal reject', async function () {
    let goalUuid = createUuid();
    let goalOrder = 10;

    function goalCallback(goal) {
      assert.strictEqual(goal.order, goalOrder);
      return rclnodejs.GoalResponse.REJECT;
    }

    function handleAcceptedCallback() {
      // This method should not be called.
      assert.ok(false);
    }

    let server = new rclnodejs.ActionServer(
      node,
      fibonacci,
      'fibonacci',
      executeCallback,
      goalCallback,
      handleAcceptedCallback
    );

    let goal = new Fibonacci.Goal();
    goal.order = goalOrder;

    await client.waitForServer(1000);

    const handle = await client.sendGoal(goal, null, goalUuid);
    assert.ok(!handle.accepted);

    server.destroy();
  });

  it('Test invalid goal callback return type', async function () {
    let goalUuid = createUuid();
    let goalOrder = 10;

    function goalCallback() {
      // Invalid return type
      return 'Invalid';
    }

    let server = new rclnodejs.ActionServer(
      node,
      fibonacci,
      'fibonacci',
      executeCallback,
      goalCallback
    );

    let goal = new Fibonacci.Goal();
    goal.order = goalOrder;

    await client.waitForServer(1000);

    const handle = await client.sendGoal(goal, null, goalUuid);
    assert.ok(!handle.accepted);

    server.destroy();
  });

  it('Test multi goal accept', async function () {
    let server = new rclnodejs.ActionServer(
      node,
      fibonacci,
      'fibonacci',
      executeCallback
    );

    let goal = new Fibonacci.Goal();

    await client.waitForServer(1000);

    const [handle1, handle2, handle3] = await Promise.all([
      client.sendGoal(goal),
      client.sendGoal(goal),
      client.sendGoal(goal),
    ]);

    assert.ok(handle1.accepted);
    assert.ok(handle2.accepted);
    assert.ok(handle3.accepted);

    server.destroy();
  });

  it('Test duplicate goal', async function () {
    let goalUuid = createUuid();
    let server = new rclnodejs.ActionServer(
      node,
      fibonacci,
      'fibonacci',
      executeCallback
    );

    let goal = new Fibonacci.Goal();

    await client.waitForServer(1000);

    const [handle1, handle2] = await Promise.all([
      client.sendGoal(goal, null, goalUuid),
      client.sendGoal(goal, null, goalUuid),
    ]);

    // Only one should have been accepted
    assert.notStrictEqual(handle1.accepted, handle2.accepted);

    server.destroy();
  });

  it('Test cancel goal accept', async function () {
    let goalOrder = 10;

    async function executeCancelCallback(goalHandle) {
      await assertUtils.createDelay(100);

      assert.ok(goalHandle.isCancelRequested);
      goalHandle.canceled();

      return new Fibonacci.Result();
    }

    function cancelCallback() {
      return rclnodejs.CancelResponse.ACCEPT;
    }

    let server = new rclnodejs.ActionServer(
      node,
      fibonacci,
      'fibonacci',
      executeCancelCallback,
      null,
      null,
      cancelCallback
    );

    let goal = new Fibonacci.Goal();
    goal.order = goalOrder;

    await client.waitForServer(1000);

    const handle = await client.sendGoal(goal);
    assert.ok(handle.accepted);

    let result = await handle.cancelGoal();
    assert.ok(result);
    assert.strictEqual(result.goals_canceling.length, 1);

    // executeCallback is async, so let it finish
    await handle.getResult();

    server.destroy();
  });

  it('Test cancel goal reject', async function () {
    let goalOrder = 10;

    async function executeCancelCallback(goalHandle) {
      await assertUtils.createDelay(50);

      assert.ok(!goalHandle.isCancelRequested);
      // Note that this will log errors to the console, which is expected
      goalHandle.canceled();

      return new Fibonacci.Result();
    }

    function cancelCallback() {
      return rclnodejs.CancelResponse.REJECT;
    }

    let server = new rclnodejs.ActionServer(
      node,
      fibonacci,
      'fibonacci',
      executeCancelCallback,
      null,
      null,
      cancelCallback
    );

    let goal = new Fibonacci.Goal();
    goal.order = goalOrder;

    await client.waitForServer(1000);

    const handle = await client.sendGoal(goal);
    assert.ok(handle.accepted);

    let result = await handle.cancelGoal();
    assert.ok(result);
    assert.strictEqual(result.goals_canceling.length, 0);

    // executeCallback is async, so let it finish
    await handle.getResult();

    server.destroy();
  });

  it('Test cancel deferred goal', async function () {
    let goalOrder = 10;
    let serverGoalHandle;

    function executeCancelCallback(goalHandle) {
      assert.ok(goalHandle.isCancelRequested);
      goalHandle.canceled();

      return new Fibonacci.Result();
    }

    function handleAcceptedCallback(goalHandle) {
      serverGoalHandle = goalHandle;
    }

    function cancelCallback() {
      return rclnodejs.CancelResponse.ACCEPT;
    }

    let server = new rclnodejs.ActionServer(
      node,
      fibonacci,
      'fibonacci',
      executeCancelCallback,
      null,
      handleAcceptedCallback,
      cancelCallback
    );

    let goal = new Fibonacci.Goal();
    goal.order = goalOrder;

    await client.waitForServer(1000);

    const handle = await client.sendGoal(goal);
    assert.ok(handle.accepted);

    let result = await handle.cancelGoal();
    assert.ok(result);
    assert.strictEqual(result.goals_canceling.length, 1);

    // Execute the goal
    serverGoalHandle.execute();

    result = await handle.getResult();
    assert.strictEqual(result.status, GoalStatus.STATUS_CANCELED);
    assert.strictEqual(serverGoalHandle.status, GoalStatus.STATUS_CANCELED);

    server.destroy();
  });

  it('Test execute succeed', async function () {
    const testSequence = [1, 1, 2, 3, 5];

    function executeSuceedCallback(goalHandle) {
      assert.strictEqual(goalHandle.status, GoalStatus.STATUS_EXECUTING);

      const resultMessage = new Fibonacci.Result();
      resultMessage.sequence = testSequence;
      goalHandle.succeed();

      return resultMessage;
    }

    let server = new rclnodejs.ActionServer(
      node,
      fibonacci,
      'fibonacci',
      executeSuceedCallback
    );

    let goal = new Fibonacci.Goal();

    await client.waitForServer(1000);
    const handle = await client.sendGoal(goal);
    assert.ok(handle.accepted);

    let result = await handle.getResult();
    assert.ok(result);
    assert.ok(result.status, GoalStatus.STATUS_SUCCEEDED);
    assert.ok(deepEqual(result.result.sequence, testSequence));

    server.destroy();
  });

  it('Test execute abort', async function () {
    const testSequence = [1, 1, 2, 3, 5];

    function executeSuceedCallback(goalHandle) {
      assert.strictEqual(goalHandle.status, GoalStatus.STATUS_EXECUTING);

      const resultMessage = new Fibonacci.Result();
      resultMessage.sequence = testSequence;
      goalHandle.abort();

      return resultMessage;
    }

    let server = new rclnodejs.ActionServer(
      node,
      fibonacci,
      'fibonacci',
      executeSuceedCallback
    );

    let goal = new Fibonacci.Goal();

    await client.waitForServer(1000);
    const handle = await client.sendGoal(goal);
    assert.ok(handle.accepted);

    let result = await handle.getResult();
    assert.ok(result);
    assert.ok(result.status, GoalStatus.STATUS_ABORTED);
    assert.ok(deepEqual(result.result.sequence, testSequence));

    server.destroy();
  });

  it('Test execute no terminal state', async function () {
    const testSequence = [1, 1, 2, 3, 5];

    function executeSuceedCallback(goalHandle) {
      // Don't set goal handle state
      assert.strictEqual(goalHandle.status, GoalStatus.STATUS_EXECUTING);

      const resultMessage = new Fibonacci.Result();
      resultMessage.sequence = testSequence;

      return resultMessage;
    }

    let server = new rclnodejs.ActionServer(
      node,
      fibonacci,
      'fibonacci',
      executeSuceedCallback
    );

    let goal = new Fibonacci.Goal();

    await client.waitForServer(1000);
    const handle = await client.sendGoal(goal);
    assert.ok(handle.accepted);

    let result = await handle.getResult();
    assert.ok(result);
    // Goal status should default to aborted
    assert.ok(result.status, GoalStatus.STATUS_ABORTED);
    assert.ok(deepEqual(result.result.sequence, testSequence));

    server.destroy();
  });

  it('Test execute throws', async function () {
    function executeErrorCallback() {
      // Note that this will log errors to the console, which is expected
      throw new Error();
    }

    let server = new rclnodejs.ActionServer(
      node,
      fibonacci,
      'fibonacci',
      executeErrorCallback
    );

    let goal = new Fibonacci.Goal();

    await client.waitForServer(1000);
    const handle = await client.sendGoal(goal);
    assert.ok(handle.accepted);

    let result = await handle.getResult();
    assert.ok(result);
    // Goal status should default to aborted
    assert.ok(result.status, GoalStatus.STATUS_ABORTED);
    assert.ok(deepEqual(result.result.sequence, []));

    server.destroy();
  });

  it('Test expire goals none', async function () {
    let server = new rclnodejs.ActionServer(
      node,
      fibonacci,
      'fibonacci',
      executeCallback,
      null,
      null,
      null,
      { resultTimeout: 1 }
    );

    let goal = new Fibonacci.Goal();

    await client.waitForServer(1000);
    await client.sendGoal(goal);

    await assertUtils.createDelay(500);
    assert.ok(server._goalHandles.size, 1);

    server.destroy();
  });

  it('Test expire goals single', async function () {
    let server = new rclnodejs.ActionServer(
      node,
      fibonacci,
      'fibonacci',
      executeCallback,
      null,
      null,
      null,
      { resultTimeout: 1 }
    );

    let goal = new Fibonacci.Goal();

    await client.waitForServer(1000);
    await client.sendGoal(goal);

    await assertUtils.createDelay(500);
    assert.ok(server._goalHandles.size, 1);

    await assertUtils.createDelay(1000);
    assert.ok(server._goalHandles.size, 0);

    server.destroy();
  });

  it('Test expire goals multi', async function () {
    let server = new rclnodejs.ActionServer(
      node,
      fibonacci,
      'fibonacci',
      executeCallback,
      null,
      null,
      null,
      { resultTimeout: 1 }
    );

    let goal = new Fibonacci.Goal();

    await client.waitForServer(1000);
    await Promise.all([
      client.sendGoal(goal),
      client.sendGoal(goal),
      client.sendGoal(goal),
    ]);

    await assertUtils.createDelay(500);
    assert.ok(server._goalHandles.size, 3);

    await assertUtils.createDelay(1000);
    assert.ok(server._goalHandles.size, 0);

    server.destroy();
  });

  it('Test feedback', async function () {
    const sequence = [1, 1, 2, 3];

    function executeFeedbackCallback(goalHandle) {
      const feedback = new Fibonacci.Feedback();
      feedback.sequence = sequence;

      goalHandle.publishFeedback(feedback);
      goalHandle.succeed();

      return new Fibonacci.Result();
    }

    let server = new rclnodejs.ActionServer(
      node,
      fibonacci,
      'fibonacci',
      executeFeedbackCallback
    );

    let goal = new Fibonacci.Goal();

    await client.waitForServer(1000);
    let feedbackMessage;
    const handle = await client.sendGoal(
      goal,
      (feedback) => (feedbackMessage = feedback)
    );
    assert.ok(handle.accepted);

    await assertUtils.createDelay(50);

    assert.ok(feedbackMessage);
    assert.ok(deepEqual(sequence, feedbackMessage.feedback.sequence));

    server.destroy();
  });

  it('Test wrong feedback type throws', async function () {
    const sequence = [1, 1, 2, 3];

    function executeFeedbackCallback(goalHandle) {
      try {
        goalHandle.publishFeedback('Wrong type');
        // We should not get here
        fail();
      } catch (error) {
        const feedback = new Fibonacci.Feedback();
        feedback.sequence = sequence;

        goalHandle.publishFeedback(feedback);
      }
      goalHandle.succeed();

      return new Fibonacci.Result();
    }

    let server = new rclnodejs.ActionServer(
      node,
      fibonacci,
      'fibonacci',
      executeFeedbackCallback
    );

    let goal = new Fibonacci.Goal();

    await client.waitForServer(1000);
    let feedbackMessage;
    const handle = await client.sendGoal(
      goal,
      (feedback) => (feedbackMessage = feedback)
    );
    assert.ok(handle.accepted);

    await assertUtils.createDelay(50);

    assert.ok(feedbackMessage);
    assert.ok(deepEqual(sequence, feedbackMessage.feedback.sequence));

    server.destroy();
  });
});
