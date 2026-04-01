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
const assertUtils = require('./utils.js');
const rclnodejs = require('../index.js');
const { ActionUuid } = require('../index.js');
const { isActionIntrospectionSupported } = require('./utils.js');

describe('rclnodejs action client', function () {
  let node;
  let server;
  this.timeout(60 * 1000);
  let fibonacci = 'example_interfaces/action/Fibonacci';
  let Fibonacci;

  let publishFeedback = null;

  async function executeCallback(goalHandle) {
    // Delay before publishing feedback to allow the client time to process
    // the goal response and set up the content filter (if enabled).
    await assertUtils.createDelay(50);

    if (
      publishFeedback &&
      ActionUuid.fromMessage(publishFeedback).toString() ===
        ActionUuid.fromMessage(goalHandle.goalId).toString()
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

    let goalUuid = ActionUuid.randomMessage();

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

    let goal1Uuid = ActionUuid.randomMessage();
    let goal2Uuid = ActionUuid.randomMessage();

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
      ActionUuid.fromMessage(result.goals_canceling[0].goal_id).toString(),
      ActionUuid.fromMessage(goalHandle.goalId).toString()
    );

    client.destroy();
  });

  it('Test getNumEntities', function () {
    let client = new rclnodejs.ActionClient(node, fibonacci, 'fibonacci');
    const numEntities = client.getNumEntities();
    assert.strictEqual(numEntities.subscriptionsNumber, 2);
    assert.strictEqual(numEntities.guardConditionsNumber, 0);
    assert.strictEqual(numEntities.timersNumber, 0);
    assert.strictEqual(numEntities.clientsNumber, 3);
    assert.strictEqual(numEntities.servicesNumber, 0);

    client.destroy();
  });

  it('Configure introspection', function () {
    if (!isActionIntrospectionSupported()) {
      this.skip();
    }
    let client = new rclnodejs.ActionClient(node, fibonacci, 'fibonacci');
    const ServiceIntrospectionStates = rclnodejs.ServiceIntrospectionStates;
    const QOS = rclnodejs.QoS.profileSystemDefault;
    client.configureIntrospection(
      node.getClock(),
      QOS,
      ServiceIntrospectionStates.CONTENTS
    );
  });

  describe('enableFeedbackMsgOptimization', function () {
    const nativeLoader = require('../lib/native_loader.js');
    const isFeedbackFilterSupported = () =>
      typeof nativeLoader.actionConfigureFeedbackSubFilterAddGoalId ===
      'function';

    it('Test option defaults to false', function () {
      let client = new rclnodejs.ActionClient(node, fibonacci, 'fibonacci');
      assert.strictEqual(client._enableFeedbackMsgOptimization, false);
      client.destroy();
    });

    it('Test option can be set to true', function () {
      let client = new rclnodejs.ActionClient(node, fibonacci, 'fibonacci', {
        enableFeedbackMsgOptimization: true,
      });
      // Only enabled when native API exists
      if (isFeedbackFilterSupported()) {
        assert.strictEqual(client._enableFeedbackMsgOptimization, true);
      } else {
        assert.strictEqual(client._enableFeedbackMsgOptimization, false);
      }
      client.destroy();
    });

    it('Test does not affect normal feedback reception', async function () {
      let client = new rclnodejs.ActionClient(node, fibonacci, 'fibonacci', {
        enableFeedbackMsgOptimization: true,
      });

      let feedbackCallback = sinon.spy(function (feedback) {
        assert.ok(feedback);
      });

      let goalUuid = ActionUuid.randomMessage();
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
      assert.ok(feedbackCallback.calledOnce);

      client.destroy();
    });

    // Verify that enabling the content filter optimization does not break
    // feedback delivery when multiple goals are active concurrently.
    it('Test multiple goals with optimization enabled still receive feedback correctly', async function () {
      let client = new rclnodejs.ActionClient(node, fibonacci, 'fibonacci', {
        enableFeedbackMsgOptimization: true,
      });

      let goal1Uuid = ActionUuid.randomMessage();
      let goal2Uuid = ActionUuid.randomMessage();

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

      // Only first goal should have received feedback
      assert.ok(feedback1Callback.calledOnce);
      assert.ok(feedback2Callback.notCalled);

      client.destroy();
    });

    it('Test cancel goal then send new goal', async function () {
      let client = new rclnodejs.ActionClient(node, fibonacci, 'fibonacci', {
        enableFeedbackMsgOptimization: true,
      });

      let result = await client.waitForServer(2000);
      assert.ok(result);

      let goalHandle = await client.sendGoal(new Fibonacci.Goal());
      assert.ok(goalHandle.isAccepted());

      result = await goalHandle.cancelGoal();
      assert.ok(result);

      assert.strictEqual(
        ActionUuid.fromMessage(result.goals_canceling[0].goal_id).toString(),
        ActionUuid.fromMessage(goalHandle.goalId).toString()
      );

      // Send another goal after cancel - should still work
      let goalHandle2 = await client.sendGoal(new Fibonacci.Goal());
      assert.ok(goalHandle2.isAccepted());

      let result2 = await goalHandle2.getResult();
      assert.ok(result2);

      client.destroy();
    });

    it('Test send multiple goals (3)', async function () {
      let client = new rclnodejs.ActionClient(node, fibonacci, 'fibonacci', {
        enableFeedbackMsgOptimization: true,
      });

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

    it('Test handles more than 6 goals gracefully', async function () {
      // The DDS content filter limit is 6 concurrent goals (100 params / 16 per goal).
      // When exceeded, optimization auto-disables but goals should still work.
      let client = new rclnodejs.ActionClient(node, fibonacci, 'fibonacci', {
        enableFeedbackMsgOptimization: true,
      });

      let feedbackCallback = sinon.spy();
      let goalUuid = ActionUuid.randomMessage();
      publishFeedback = goalUuid;

      let result = await client.waitForServer(2000);
      assert.ok(result);

      // Send 7 goals sequentially - exceeds the 6 goal content filter limit
      let handles = [];
      for (let i = 0; i < 7; i++) {
        let uuid = i === 0 ? goalUuid : undefined;
        let cb = i === 0 ? feedbackCallback : undefined;
        let h = await client.sendGoal(new Fibonacci.Goal(), cb, uuid);
        assert.ok(h.isAccepted());
        handles.push(h);
      }

      // Wait for all results
      for (const h of handles) {
        let r = await h.getResult();
        assert.ok(r);
      }

      client.destroy();
    });
  });
});
