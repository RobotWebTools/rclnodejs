// Copyright (c) 2026 The Robot Web Tools Contributors. All rights reserved.
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

describe('QoS overriding options', function () {
  let node;
  this.timeout(60 * 1000);

  before(function () {
    return rclnodejs.init();
  });

  after(function () {
    rclnodejs.shutdown();
  });

  beforeEach(function () {
    node = rclnodejs.createNode('qos_override_test_node');
  });

  afterEach(function () {
    node.destroy();
  });

  it('QoSPolicyKind enum has expected values', function () {
    const { QoSPolicyKind } = rclnodejs;
    assert.ok(QoSPolicyKind.HISTORY);
    assert.ok(QoSPolicyKind.DEPTH);
    assert.ok(QoSPolicyKind.RELIABILITY);
    assert.ok(QoSPolicyKind.DURABILITY);
    assert.ok(QoSPolicyKind.LIVELINESS);
    assert.ok(QoSPolicyKind.AVOID_ROS_NAMESPACE_CONVENTIONS);
  });

  it('QoSOverridingOptions.withDefaultPolicies() creates correct policies', function () {
    const opts = rclnodejs.QoSOverridingOptions.withDefaultPolicies();
    assert.strictEqual(opts.policyKinds.length, 3);
    assert.ok(opts.policyKinds.includes(rclnodejs.QoSPolicyKind.HISTORY));
    assert.ok(opts.policyKinds.includes(rclnodejs.QoSPolicyKind.DEPTH));
    assert.ok(opts.policyKinds.includes(rclnodejs.QoSPolicyKind.RELIABILITY));
    assert.strictEqual(opts.callback, null);
    assert.strictEqual(opts.entityId, null);
  });

  it('QoSOverridingOptions with custom policies and entityId', function () {
    const opts = new rclnodejs.QoSOverridingOptions(
      [rclnodejs.QoSPolicyKind.DURABILITY],
      { entityId: 'sensor' }
    );
    assert.strictEqual(opts.policyKinds.length, 1);
    assert.strictEqual(opts.entityId, 'sensor');
  });

  it('Publisher declares QoS override parameters', function () {
    const pub = node.createPublisher('std_msgs/msg/String', 'test_pub_qos', {
      qos: new rclnodejs.QoS(
        rclnodejs.QoS.HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        10,
        rclnodejs.QoS.ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
        rclnodejs.QoS.DurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE
      ),
      qosOverridingOptions:
        rclnodejs.QoSOverridingOptions.withDefaultPolicies(),
    });

    const resolvedTopic = node.resolveTopicName('test_pub_qos');

    // Check that parameters were declared
    assert.ok(
      node.hasParameter(`qos_overrides.${resolvedTopic}.publisher.history`)
    );
    assert.ok(
      node.hasParameter(`qos_overrides.${resolvedTopic}.publisher.depth`)
    );
    assert.ok(
      node.hasParameter(`qos_overrides.${resolvedTopic}.publisher.reliability`)
    );

    // Check parameter values match the QoS
    const historyParam = node.getParameter(
      `qos_overrides.${resolvedTopic}.publisher.history`
    );
    assert.strictEqual(historyParam.value, 'keep_last');

    const depthParam = node.getParameter(
      `qos_overrides.${resolvedTopic}.publisher.depth`
    );
    assert.strictEqual(Number(depthParam.value), 10);

    const reliabilityParam = node.getParameter(
      `qos_overrides.${resolvedTopic}.publisher.reliability`
    );
    assert.strictEqual(reliabilityParam.value, 'reliable');

    node.destroyPublisher(pub);
  });

  it('Subscription declares QoS override parameters', function () {
    const sub = node.createSubscription(
      'std_msgs/msg/String',
      'test_sub_qos',
      {
        qos: new rclnodejs.QoS(
          rclnodejs.QoS.HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
          5,
          rclnodejs.QoS.ReliabilityPolicy
            .RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
          rclnodejs.QoS.DurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE
        ),
        qosOverridingOptions:
          rclnodejs.QoSOverridingOptions.withDefaultPolicies(),
      },
      () => {}
    );

    const resolvedTopic = node.resolveTopicName('test_sub_qos');

    assert.ok(
      node.hasParameter(`qos_overrides.${resolvedTopic}.subscription.depth`)
    );

    const depthParam = node.getParameter(
      `qos_overrides.${resolvedTopic}.subscription.depth`
    );
    assert.strictEqual(Number(depthParam.value), 5);

    const reliabilityParam = node.getParameter(
      `qos_overrides.${resolvedTopic}.subscription.reliability`
    );
    assert.strictEqual(reliabilityParam.value, 'best_effort');

    node.destroySubscription(sub);
  });

  it('Entity ID suffix in parameter names', function () {
    const pub = node.createPublisher('std_msgs/msg/String', 'test_entity_id', {
      qos: rclnodejs.QoS.profileDefault,
      qosOverridingOptions: new rclnodejs.QoSOverridingOptions(
        [rclnodejs.QoSPolicyKind.DEPTH],
        { entityId: 'camera' }
      ),
    });

    const resolvedTopic = node.resolveTopicName('test_entity_id');
    assert.ok(
      node.hasParameter(`qos_overrides.${resolvedTopic}.publisher_camera.depth`)
    );

    node.destroyPublisher(pub);
  });

  it('Validation callback rejects invalid QoS', function () {
    assert.throws(() => {
      node.createPublisher('std_msgs/msg/String', 'test_validate', {
        qos: new rclnodejs.QoS(
          rclnodejs.QoS.HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
          10
        ),
        qosOverridingOptions: new rclnodejs.QoSOverridingOptions(
          [rclnodejs.QoSPolicyKind.DEPTH],
          {
            callback: () => ({
              successful: false,
              reason: 'Depth must be > 100',
            }),
          }
        ),
      });
    }, /QoS override validation failed/);
  });

  it('Validation callback accepts valid QoS', function () {
    const pub = node.createPublisher(
      'std_msgs/msg/String',
      'test_validate_ok',
      {
        qos: new rclnodejs.QoS(
          rclnodejs.QoS.HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
          10
        ),
        qosOverridingOptions: new rclnodejs.QoSOverridingOptions(
          [rclnodejs.QoSPolicyKind.DEPTH],
          { callback: () => ({ successful: true }) }
        ),
      }
    );

    assert.ok(pub);
    node.destroyPublisher(pub);
  });

  it('String QoS profiles are resolved before overriding', function () {
    const pub = node.createPublisher('std_msgs/msg/String', 'test_string_qos', {
      qos: rclnodejs.QoS.profileDefault,
      qosOverridingOptions:
        rclnodejs.QoSOverridingOptions.withDefaultPolicies(),
    });

    const resolvedTopic = node.resolveTopicName('test_string_qos');
    assert.ok(
      node.hasParameter(`qos_overrides.${resolvedTopic}.publisher.depth`)
    );

    node.destroyPublisher(pub);
  });

  it('No qosOverridingOptions means no parameters declared', function () {
    const pub = node.createPublisher(
      'std_msgs/msg/String',
      'test_no_override',
      {
        qos: rclnodejs.QoS.profileDefault,
      }
    );

    const resolvedTopic = node.resolveTopicName('test_no_override');
    assert.ok(
      !node.hasParameter(`qos_overrides.${resolvedTopic}.publisher.depth`)
    );

    node.destroyPublisher(pub);
  });

  it('Parameter override changes the actual QoS used', function () {
    // Equivalent to running: node my_app.js --ros-args -p "qos_overrides./test_override_applied.subscription.depth:=1"
    // In tests, we use NodeOptions.parameterOverrides since rclnodejs.init()
    // (which processes --ros-args from process.argv) is already called.
    // Both mechanisms populate the same _parameterOverrides map on the node.
    const overrideNode = new rclnodejs.Node(
      'qos_override_applied_node',
      '',
      rclnodejs.Context.defaultContext(),
      {
        startParameterServices: false,
        parameterOverrides: [
          new rclnodejs.Parameter(
            'qos_overrides./test_override_applied.subscription.depth',
            rclnodejs.ParameterType.PARAMETER_INTEGER,
            1
          ),
        ],
      }
    );

    const qos = new rclnodejs.QoS(
      rclnodejs.QoS.HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
      10, // code default: depth=10
      rclnodejs.QoS.ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
      rclnodejs.QoS.DurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE
    );

    const sub = overrideNode.createSubscription(
      'std_msgs/msg/String',
      'test_override_applied',
      {
        qos: qos,
        qosOverridingOptions:
          rclnodejs.QoSOverridingOptions.withDefaultPolicies(),
      },
      () => {}
    );

    // The parameter override (depth=1) should have been applied to the QoS
    const depthParam = overrideNode.getParameter(
      'qos_overrides./test_override_applied.subscription.depth'
    );
    assert.strictEqual(Number(depthParam.value), 1);

    // The QoS object should have been mutated in-place
    assert.strictEqual(qos.depth, 1);

    overrideNode.destroySubscription(sub);
    overrideNode.destroy();
  });
});
