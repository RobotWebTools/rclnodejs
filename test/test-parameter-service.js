/* eslint-disable camelcase */
// Copyright (c) Wayne Parrott All rights reserved.
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

const {
  ParameterType,
  Parameter,
  ParameterDescriptor,
  PARAMETER_SEPARATOR,
} = require('../lib/parameter.js');

const PARAM_REL_TOL = 1e-6;

const Node = rclnodejs.Node;

const PARAMETER_EVENT_MSG_TYPE = 'rcl_interfaces/msg/ParameterEvent';
const PARAMETER_EVENT_TOPIC = 'parameter_events';

const STD_WAIT = 500; // ms to delay/wait

describe('Parameter_server tests', function() {
  const argv = [
    '--ros-args',
    '-p',
    'test_node:p1:=foobar',
    '-p',
    'test_node:p2:=123',
    '-p',
    'test_node:A.p3:=true',
    '-p',
    'test_node:A.B.p4:=666.6',
  ];
  const NODE_NAME = 'test_node';
  const CLIENT_NODE_NAME = 'client_test_node';

  let node;
  let clientNode;
  this.timeout(60 * 1000);

  beforeEach(async function() {
    await rclnodejs.init();

    node = rclnodejs.createNode(NODE_NAME);

    node.declareParameter(
      new Parameter('p1', ParameterType.PARAMETER_STRING, 'helloworld'),
      new ParameterDescriptor(
        'p1',
        ParameterType.PARAMETER_STRING,
        'hello world msg',
        false
      )
    );
    node.declareParameter(
      new Parameter('p2', ParameterType.PARAMETER_INTEGER, 123),
      new ParameterDescriptor('p2', ParameterType.PARAMETER_INTEGER)
    );
    node.declareParameter(
      new Parameter('A.p3', ParameterType.PARAMETER_BOOL, true),
      new ParameterDescriptor(
        'A.p3',
        ParameterType.PARAMETER_BOOL,
        undefined,
        true
      )
    );
    node.declareParameter(
      new Parameter('A.B.p4', ParameterType.PARAMETER_INTEGER_ARRAY, [1, 2, 3]),
      new ParameterDescriptor(
        'A.B.p4',
        ParameterType.PARAMETER_INTEGER_ARRAY,
        'array of ints',
        false
      )
    );

    clientNode = rclnodejs.createNode(CLIENT_NODE_NAME);

    rclnodejs.spin(node);
    rclnodejs.spin(clientNode);
  });

  afterEach(function() {
    clientNode.destroy();
    node.destroy();
    rclnodejs.shutdown();
  });

  it('List parameters', async function() {
    const client = clientNode.createClient(
      'rcl_interfaces/srv/ListParameters',
      'test_node/list_parameters'
    );
    await client.waitForService();

    const request = new (rclnodejs.require(
      'rcl_interfaces/srv/ListParameters'
    ).Request)();
    request.depth = 1;
    request.prefixes = [];

    let success = false;
    client.sendRequest(request, response => {
      const result = response.result;
      assert.equal(result.names.length, 2);
      assert.ok(result.names.includes('p1'));
      assert.ok(result.names.includes('p2'));
      success = true;
    });

    await assertUtils.createDelay(STD_WAIT);
    assert.ok(success);
  });

  it('List parameters with prefixes, depth=2', async function() {
    const client = clientNode.createClient(
      'rcl_interfaces/srv/ListParameters',
      'test_node/list_parameters'
    );
    await client.waitForService();

    const request = new (rclnodejs.require(
      'rcl_interfaces/srv/ListParameters'
    ).Request)();
    request.depth = 2;
    request.prefixes = ['A'];

    let success = false;
    client.sendRequest(request, response => {
      const result = response.result;

      assert.equal(result.names.length, 1);
      assert.ok(result.prefixes.includes('A'));
      assert.ok(result.names.includes('A.p3'));
      success = true;
    });

    await assertUtils.createDelay(STD_WAIT);
    assert.ok(success);
  });

  it('List parameters with prefixes, depth=3', async function() {
    const client = clientNode.createClient(
      'rcl_interfaces/srv/ListParameters',
      'test_node/list_parameters'
    );
    await client.waitForService();

    const request = new (rclnodejs.require(
      'rcl_interfaces/srv/ListParameters'
    ).Request)();
    request.depth = 3;
    request.prefixes = ['A'];

    let success = false;
    client.sendRequest(request, response => {
      const result = response.result;

      assert.equal(result.names.length, 2);
      assert.ok(result.names.includes('A.p3'));
      assert.ok(result.names.includes('A.B.p4'));
      assert.ok(result.prefixes.includes('A'));
      assert.ok(result.prefixes.includes('A.B'));
      success = true;
    });

    await assertUtils.createDelay(STD_WAIT);
    assert.ok(success);
  });

  it('Describe parameters', async function() {
    const client = clientNode.createClient(
      'rcl_interfaces/srv/DescribeParameters',
      'test_node/describe_parameters'
    );
    await client.waitForService();

    const request = new (rclnodejs.require(
      'rcl_interfaces/srv/DescribeParameters'
    ).Request)();
    request.names = ['p1', 'p2'];

    let success = false;
    client.sendRequest(request, response => {
      // process service response
      assert.equal(response.descriptors.length, 2);

      for (const descriptor of response.descriptors) {
        if (descriptor.name == 'p1') {
          assert.equal(descriptor.type, ParameterType.PARAMETER_STRING);
          assert.equal(descriptor.description, 'hello world msg');
        } else if (descriptor.name == 'p2') {
          assert.equal(descriptor.type, ParameterType.PARAMETER_INTEGER);
        } else {
          assert.ok(false, `returned unexpected descriptor ${descriptor.name}`);
        }
      }

      success = true;
    });

    await assertUtils.createDelay(STD_WAIT);
    assert.ok(success);
  });

  it('Get_parameters', async function() {
    const client = clientNode.createClient(
      'rcl_interfaces/srv/GetParameters',
      'test_node/get_parameters'
    );
    await client.waitForService();

    const request = new (rclnodejs.require(
      'rcl_interfaces/srv/GetParameters'
    ).Request)();
    request.names = ['p1', 'A.p3'];

    let success = false;
    client.sendRequest(request, response => {
      assert.equal(response.values.length, 2);

      // p1 value
      const p1 = Parameter.fromParameterMessage({
        name: 'p1',
        value: response.values[0],
      });
      assert.equal(p1.type, ParameterType.PARAMETER_STRING);
      assert.equal(p1.value, 'helloworld');

      // A.p3 value
      const p3 = Parameter.fromParameterMessage({
        name: 'p1',
        value: response.values[1],
      });
      assert.equal(p3.type, ParameterType.PARAMETER_BOOL);
      assert.equal(p3.value, true);

      success = true;
    });

    await assertUtils.createDelay(STD_WAIT);
    assert.ok(success);
  });

  it('Set parameters message', async function() {
    let completed = false;

    const client = clientNode.createClient(
      'rcl_interfaces/srv/SetParameters',
      'test_node/set_parameters'
    );
    await client.waitForService();

    const p1 = node.getParameter('p1');
    const p1a = new Parameter(p1.name, p1.type, 'abcdef');
    const request = new (rclnodejs.require(
      'rcl_interfaces/srv/SetParameters'
    ).Request)();
    request.parameters = [p1a.toParameterMessage()];

    client.sendRequest(request, response => {
      assert.equal(node.getParameter('p1').value, 'abcdef');
      completed = true;
    });

    await assertUtils.createDelay(STD_WAIT);
    assert.ok(completed, 'Waiting on service response.');
  });

  it('Set_parameters_atomically', async function() {
    let completed = false;

    const client = clientNode.createClient(
      'rcl_interfaces/srv/SetParametersAtomically',
      'test_node/set_parameters_atomically'
    );
    await client.waitForService();

    const p1 = node.getParameter('p1');
    const p1a = new Parameter(p1.name, p1.type, 'abcdef');
    const request = new (rclnodejs.require(
      'rcl_interfaces/srv/SetParametersAtomically'
    ).Request)();
    request.parameters = [p1a.toParameterMessage()];

    client.sendRequest(request, response => {
      assert.equal(node.getParameter('p1').value, 'abcdef');
      completed = true;
    });

    await assertUtils.createDelay(STD_WAIT);
    assert.ok(completed, 'Waiting on service response.');
  });

  it('Parameter events', async function() {
    let eventCount = 0;

    const subscription = clientNode.createSubscription(
      PARAMETER_EVENT_MSG_TYPE,
      PARAMETER_EVENT_TOPIC,

      parameterEvent => {
        eventCount++;

        if (parameterEvent.new_parameters.length > 0) {
          const parameter = Parameter.fromParameterMessage(
            parameterEvent.new_parameters[0]
          );
          assert.equal(parameter.name, 'x1');
          assert.equal(parameter.value, 'abc');
        } else if (parameterEvent.changed_parameters.length > 0) {
          const parameter = Parameter.fromParameterMessage(
            parameterEvent.changed_parameters[0]
          );
          assert.equal(parameter.name, 'x1');
          assert.equal(parameter.value, 'def');
          assert.equal(node.getParameter('x1').value, parameter.value);
        } else if (parameterEvent.deleted_parameters.length > 0) {
          const parameter = Parameter.fromParameterMessage(
            parameterEvent.deleted_parameters[0]
          );
          assert.equal(parameter.name, 'x1');
          assert.ok(!node.hasParameter('x1'));
        }
      }
    );

    await assertUtils.createDelay(STD_WAIT);

    node.declareParameter(
      new Parameter('x1', ParameterType.PARAMETER_STRING, 'abc'),
      undefined,
      false
    );
    await assertUtils.createDelay(STD_WAIT);

    node.setParameter(
      new Parameter('x1', ParameterType.PARAMETER_STRING, 'def')
    );
    await assertUtils.createDelay(STD_WAIT);

    node.setParameter(new Parameter('x1', ParameterType.PARAMETER_NOT_SET));
    await assertUtils.createDelay(STD_WAIT);

    assert.equal(
      eventCount,
      3,
      `Expected 3 parameter-events, received ${eventCount} events.`
    );
  });
});
