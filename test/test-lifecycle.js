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
const childProcess = require('child_process');
const assertUtils = require('./utils.js');
const assertThrowsError = assertUtils.assertThrowsError;

const NODE_NAME = 'lifecycle_node';

let StateInterface;
let TransitionInterface;

function verifyAvailableStates(states) {
  assert.equal(states.length, 11);

  // Primary States
  assert.equal(states[0].id, 0);
  assert.equal(states[1].id, StateInterface.PRIMARY_STATE_UNCONFIGURED);
  assert.equal(states[2].id, StateInterface.PRIMARY_STATE_INACTIVE);
  assert.equal(states[3].id, StateInterface.PRIMARY_STATE_ACTIVE);
  assert.equal(states[4].id, StateInterface.PRIMARY_STATE_FINALIZED);

  // Transition States
  assert.equal(states[5].id, StateInterface.TRANSITION_STATE_CONFIGURING);
  assert.equal(states[6].id, StateInterface.TRANSITION_STATE_CLEANINGUP);
  assert.equal(states[7].id, StateInterface.TRANSITION_STATE_SHUTTINGDOWN);
  assert.equal(states[8].id, StateInterface.TRANSITION_STATE_ACTIVATING);
  assert.equal(states[9].id, StateInterface.TRANSITION_STATE_DEACTIVATING);
  assert.equal(states[10].id, StateInterface.TRANSITION_STATE_ERRORPROCESSING);
}

describe('LifecycleNode test suite', function () {
  let node;
  this.timeout(60 * 1000);

  before(async function () {
    await rclnodejs.init();
    StateInterface = rclnodejs.createMessage(
      'lifecycle_msgs/msg/State'
    ).constructor;
    TransitionInterface = rclnodejs.createMessage(
      'lifecycle_msgs/msg/Transition'
    ).constructor;
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

  it('lifecycleNode initial state', function () {
    const state = node.currentState;
    assert.equal(state.id, StateInterface.PRIMARY_STATE_UNCONFIGURED);
  });

  it('lifecycleNode transitions', function () {
    assert.equal(
      node.currentState.id,
      StateInterface.PRIMARY_STATE_UNCONFIGURED
    );

    let state = node.configure();
    assert.equal(node.currentState.id, StateInterface.PRIMARY_STATE_INACTIVE);
    assert.equal(state.id, StateInterface.PRIMARY_STATE_INACTIVE);

    state = node.activate();
    assert.equal(node.currentState.id, StateInterface.PRIMARY_STATE_ACTIVE);
    assert.equal(state.id, StateInterface.PRIMARY_STATE_ACTIVE);

    state = node.deactivate();
    assert.equal(node.currentState.id, StateInterface.PRIMARY_STATE_INACTIVE);
    assert.equal(state.id, StateInterface.PRIMARY_STATE_INACTIVE);

    state = node.cleanup();
    assert.equal(
      node.currentState.id,
      StateInterface.PRIMARY_STATE_UNCONFIGURED
    );
    assert.equal(state.id, StateInterface.PRIMARY_STATE_UNCONFIGURED);

    state = node.shutdown();
    assert.equal(node.currentState.id, StateInterface.PRIMARY_STATE_FINALIZED);
    assert.equal(state.id, StateInterface.PRIMARY_STATE_FINALIZED);
  });

  it('lifecycleNode getAvailableStates', function () {
    const states = node.availableStates;
    verifyAvailableStates(states);
  });

  it('lifecycleNode callback registration and invoke', function () {
    let cbCnt = 0;

    let cb = (prevState) => {
      cbCnt++;
      return rclnodejs.lifecycle.CallbackReturnCode.SUCCESS;
    };

    node.registerOnConfigure(cb);
    node.registerOnActivate(cb);
    node.registerOnDeactivate(cb);
    node.registerOnCleanup(cb);
    node.registerOnShutdown(cb);
    node.registerOnError(cb);

    node.configure();
    node.activate();
    node.deactivate();
    node.cleanup();
    node.shutdown();

    assert.equal(cbCnt, 5);
  });

  it('lifecycleNode fail transition', function () {
    assert.throws(node.activate);
  });

  it('lifecycleNode onError callback', function () {
    let failCb = (prevState) => {
      return rclnodejs.lifecycle.CallbackReturnCode.ERROR;
    };

    node.registerOnConfigure(failCb);

    let errorCbCnt = 0;
    node.registerOnError((prevState) => errorCbCnt++);

    node.configure();
    assert.equal(errorCbCnt, 1);
  });

  it('lifecycle event publisher', async function () {
    let eventCnt = 0;

    let subscription = node.createSubscription(
      'lifecycle_msgs/msg/TransitionEvent',
      '~/transition_event',
      (msg) => {
        eventCnt++;
      }
    );

    node.configure();
    await assertUtils.createDelay(1000);

    assert.equal(eventCnt, 2);
  });

  it('LifecycleNode srv/GetState', async function () {
    node.configure();

    let client = node.createClient(
      'lifecycle_msgs/srv/GetState',
      '~/get_state'
    );

    let currentState;
    client.waitForService(1000).then((result) => {
      if (!result) {
        assert.fail('Error: GetState service not available');
      }
      client.sendRequest({}, (response) => {
        currentState = response.current_state;
      });
    });

    await assertUtils.createDelay(1000);

    assert.ok(currentState);
    assert.equal(currentState.id, StateInterface.PRIMARY_STATE_INACTIVE);
  });

  it('LifecycleNode srv/GetAvailableStates', async function () {
    let client = node.createClient(
      'lifecycle_msgs/srv/GetAvailableStates',
      '~/get_available_states'
    );

    let states;
    client.waitForService(1000).then((result) => {
      if (!result) {
        assert.fail('Error: GetAvailableStates service not available');
      }
      client.sendRequest({}, (response) => {
        states = response.available_states;
      });
    });

    await assertUtils.createDelay(1000);

    assert.ok(states);
    verifyAvailableStates(states);
  });

  it('LifecycleNode srv/GetAvailableTransitions', async function () {
    let client = node.createClient(
      'lifecycle_msgs/srv/GetAvailableTransitions',
      '~/get_available_transitions'
    );

    let transitions;
    client.waitForService(1000).then((result) => {
      if (!result) {
        assert.fail('Error: GetAvailableTransitions service not available');
      }
      client.sendRequest({}, (response) => {
        transitions = response.available_transitions;
      });
    });

    await assertUtils.createDelay(1000);

    assert.ok(transitions);
    assert.strictEqual(transitions.length, 2);
    assert.ok(
      transitions[0].transition.id === 1 || // configure
        transitions[0].transition.id === 5
    ); // shutdown
    assert.ok(
      transitions[1].transition.id === 1 || // configure
        transitions[1].transition.id === 5
    ); // shutdown

    node.configure();

    transitions = node.availableTransitions;
    assert.ok(transitions);
    assert.strictEqual(transitions.length, 3);
    assert.ok(
      transitions[0].transition.id === 2 || // cleanup
        transitions[0].transition.id === 3 || // activate
        transitions[0].transition.id === 6
    ); // shutdown
    assert.ok(
      transitions[1].transition.id === 2 || // cleanup
        transitions[1].transition.id === 3 || // activate
        transitions[1].transition.id === 6
    ); // shutdown
    assert.ok(
      transitions[1].transition.id === 2 || // cleanup
        transitions[1].transition.id === 3 || // activate
        transitions[1].transition.id === 6
    ); // shutdown
  });

  it('LifecycleNode srv/ChangeState', async function () {
    let client = node.createClient(
      'lifecycle_msgs/srv/ChangeState',
      '~/change_state'
    );

    let isSuccess = false;
    client.waitForService(1000).then((result) => {
      if (!result) {
        assert.fail('Error: ChangeState service not available');
      }
      let request = {
        transition: {
          id: TransitionInterface.TRANSITION_CONFIGURE,
          label: 'configure',
        },
      };
      client.sendRequest(request, (response) => {
        isSuccess = response.success;
      });
    });

    await assertUtils.createDelay(1000);

    assert.ok(isSuccess);
  });

  it('LifecycleNode enableCommunicationsInterface', async function () {
    node.stop();

    node = rclnodejs.createLifecycleNode(NODE_NAME);

    let services = node.getServiceNamesAndTypes();
    let serviceMsgs = [];
    services.forEach((service) => {
      if (
        service.types.length &&
        service.types[0].startsWith('lifecycle_msgs')
      ) {
        serviceMsgs.push(service.types[0]);
      }
    });

    assert.equal(
      serviceMsgs.length,
      5,
      'Incomplete set of lifecycle services found'
    );
  });

  it('LifecycleNode disable enableCommunicationsInterface', async function () {
    // this test is only valid on ROS2 Galactic distro
    // TODO: refactor the version info to reusable location
    const GALACTIC_VERSION = 2105;
    const versionInfo = childProcess
      .execSync('node scripts/ros_distro.js')
      .toString('utf-8');
    const version =
      versionInfo && versionInfo.length > 0
        ? parseInt(versionInfo)
        : GALACTIC_VERSION;

    if (version < GALACTIC_VERSION) return;

    let rosDistro = process.env.ROS_DISTRO;
    let firstChar = rosDistro.charAt(0);
    let testNode = new rclnodejs.lifecycle.LifecycleNode(
      'TEST_NODE',
      undefined,
      undefined,
      undefined,
      false
    );
    testNode.spin();

    let services = testNode.getServiceNamesAndTypes();
    let serviceMsgs = [];
    services.forEach((service) => {
      if (
        service.name.startsWith('/TEST_NODE') &&
        service.types &&
        service.types.length &&
        service.types[0].startsWith('lifecycle_msgs')
      ) {
        serviceMsgs.push(service.types[0]);
      }
    });

    assert.equal(serviceMsgs.length, 0, 'Unexpected lifecycle services found');
    testNode.destroy();
  });
});
