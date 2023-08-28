// Copyright (c) 2023 Wayne Parrott. All rights reserved.
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

const childProcess = require('child_process');
const assert = require('assert');
const rclnodejs = require('../index.js');
const DistroUtils = rclnodejs.DistroUtils;
const YAML = require('yaml');

const util = require('node:util');
const exec = util.promisify(require('node:child_process').exec);

const ServiceIntrospectionStates = rclnodejs.ServiceIntrospectionStates;
const QOS = rclnodejs.QoS.profileSystemDefault;

const DELAY = 1000; // ms

//  ros2 topic echo /add_two_ints/_service_event

function isServiceIntrospectionSupported() {
  return DistroUtils.getDistroId() > DistroUtils.getDistroId('humble');
}

function runClient(client, request, delay = DELAY) {
  client.sendRequest(request, (response) => {
    // do nothing
  });

  return new Promise((resolve) => {
    setTimeout(resolve, delay);
  });
}

describe('service introspection', function () {
  this.timeout(30 * 1000);

  let node;
  let service;
  let client;
  let request;
  let serviceEventSubscriber;
  let eventQueue;

  before( function() {
    if (!isServiceIntrospectionSupported()) {
      this.skip();
    }
  });

  beforeEach(async function () {
    await rclnodejs.init();

    this.node = new rclnodejs.Node('service_example_node');

    this.service = this.node.createService(
      'example_interfaces/srv/AddTwoInts',
      'add_two_ints',
      (request, response) => {
        let result = response.template;
        result.sum = request.a + request.b;
        response.send(result);
      }
    );

    this.client = this.node.createClient(
      'example_interfaces/srv/AddTwoInts',
      'add_two_ints'
    );

    if (!(await this.client.waitForService(1000))) {
      rclnodejs.shutdown();
      throw new Error('client unable to access service');
    }

    this.request = {
      a: Math.floor(Math.random() * 100),
      b: Math.floor(Math.random() * 100),
    };

    this.eventQueue = [];
    let eventQueueRef = this.eventQueue;
    this.serviceEventSubscriber = this.node.createSubscription(
      'example_interfaces/srv/AddTwoInts_Event',
      '/add_two_ints/_service_event',
      function (event) {
        eventQueueRef.push(event);
      }
    );

    this.node.spin();
  });

  afterEach(function () {
    rclnodejs.shutdown();
  });

  it('client and service introspection: CONTENT', async function () {
    this.service.configureIntrospection(
      this.node.getClock(),
      QOS,
      ServiceIntrospectionStates.CONTENTS
    );

    this.client.configureIntrospection(
      this.node.getClock(),
      QOS,
      ServiceIntrospectionStates.CONTENTS
    );

    await runClient(this.client, this.request);

    assert.strictEqual(this.eventQueue.length, 4);
    for (let i = 0; i < 4; i++) {
      assert.strictEqual(this.eventQueue[i].info.event_type, i);
      if (i < 2) {
        assert.strictEqual(this.eventQueue[i].request.length, 1);
        assert.strictEqual(this.eventQueue[i].response.length, 0);
      } else {
        assert.strictEqual(this.eventQueue[i].request.length, 0);
        assert.strictEqual(this.eventQueue[i].response.length, 1);
      }
    }
  });

  it('service-only introspection: METADATA', async function () {
    this.service.configureIntrospection(
      this.node.getClock(),
      QOS,
      ServiceIntrospectionStates.METADATA
    );

    await runClient(this.client, this.request);

    assert.strictEqual(this.eventQueue.length, 2);

    assert.strictEqual(this.eventQueue[0].info.event_type, 1);
    assert.strictEqual(this.eventQueue[0].request.length, 0);
    assert.strictEqual(this.eventQueue[0].response.length, 0);

    assert.strictEqual(this.eventQueue[1].info.event_type, 2);
    assert.strictEqual(this.eventQueue[1].request.length, 0);
    assert.strictEqual(this.eventQueue[1].response.length, 0);
  });

  it('client-only introspection: METADATA', async function () {
    this.client.configureIntrospection(
      this.node.getClock(),
      QOS,
      ServiceIntrospectionStates.METADATA
    );

    await runClient(this.client, this.request);

    assert.strictEqual(this.eventQueue.length, 2);

    assert.strictEqual(this.eventQueue[0].info.event_type, 0);
    assert.strictEqual(this.eventQueue[0].request.length, 0);
    assert.strictEqual(this.eventQueue[0].response.length, 0);

    assert.strictEqual(this.eventQueue[1].info.event_type, 3);
    assert.strictEqual(this.eventQueue[1].request.length, 0);
    assert.strictEqual(this.eventQueue[1].response.length, 0);
  });

  it('client and service unconfigured introspection', async function () {
    await runClient(this.client, this.request);
    assert.strictEqual(this.eventQueue.length, 0);
  });

  it('client introspection: OFF, service introspection: OFF', async function () {
    this.service.configureIntrospection(
      this.node.getClock(),
      QOS,
      ServiceIntrospectionStates.OFF
    );

    this.client.configureIntrospection(
      this.node.getClock(),
      QOS,
      ServiceIntrospectionStates.OFF
    );

    await runClient(this.client, this.request);

    assert.strictEqual(this.eventQueue.length, 0);
  });

  it('client introspection: OFF, service introspection: CONTENTS', async function () {
    this.service.configureIntrospection(
      this.node.getClock(),
      QOS,
      ServiceIntrospectionStates.CONTENTS
    );

    this.client.configureIntrospection(
      this.node.getClock(),
      QOS,
      ServiceIntrospectionStates.OFF
    );

    await runClient(this.client, this.request);

    assert.strictEqual(this.eventQueue.length, 2);
    assert.strictEqual(this.eventQueue[0].info.event_type, 1);
    assert.strictEqual(this.eventQueue[1].info.event_type, 2);
  });

  it('client introspection: CONTENTS, service introspection: OFF', async function () {
    this.service.configureIntrospection(
      this.node.getClock(),
      QOS,
      ServiceIntrospectionStates.OFF
    );

    this.client.configureIntrospection(
      this.node.getClock(),
      QOS,
      ServiceIntrospectionStates.CONTENTS
    );

    await runClient(this.client, this.request);

    assert.strictEqual(this.eventQueue.length, 2);
    assert.strictEqual(this.eventQueue[0].info.event_type, 0);
    assert.strictEqual(this.eventQueue[1].info.event_type, 3);
  });
});
