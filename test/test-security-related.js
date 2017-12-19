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
const rclnodejs = require('../index.js');
const assertThrowsError = require('./utils.js').assertThrowsError;

describe('Destroying non-existent objects testing', function() {
  this.timeout(60 * 1000);

  before(function() {
    return rclnodejs.init();
  });

  after(function() {
    rclnodejs.shutdown();
  });

  it('Destroy a non-existent node', function() {
    var node = null;
    assertThrowsError(() => {
      node.destroy();
    }, TypeError, 'Cannot read property', 'Trying to destroy an empty node!');

    var node = rclnodejs.createNode('node1', '/non_existent');
    node.destroy();

    // It's OK to destroy a node multiple times 
    // so long as the resources are freed in rcl layer.
    node.destroy();
  });

  it('Destroy a non-existent publisher', function() {
    var node = rclnodejs.createNode('node2', '/non_existent');
    assertThrowsError(() => {
      node.destroyPublisher(null);
    }, TypeError, 'Invalid argument', 'Trying to destroy an empty publisher!');

    const RclString = 'std_msgs/msg/String';
    var publisher = node.createPublisher(RclString, 'chatter2');
    node.destroyPublisher(publisher);

    // OK to destroy a publisher multiple times
    node.destroyPublisher(publisher);
  });

  it('Destroy a non-existent subscription', function() {
    var node = rclnodejs.createNode('node3', '/non_existent');
    assertThrowsError(() => {
      node.destroySubscription(null);
    }, TypeError, 'Invalid argument', 'Trying to destroy an empty subscription!');

    const RclString = 'std_msgs/msg/String';
    var subscription = node.createSubscription(RclString, 'chatter3', () => {});
    node.destroySubscription(subscription);

    // OK to destroy a subscription multiple times
    node.destroySubscription(subscription);
  });

  it('Destroy a non-existent client', function() {
    var node = rclnodejs.createNode('node4', '/non_existent');
    assertThrowsError(() => {
      node.destroyClient(null);
    }, TypeError, 'Invalid argument', 'Trying to destroy an empty client!');

    const AddTwoInts = 'example_interfaces/srv/AddTwoInts';
    var client = node.createClient(AddTwoInts, 'add_two_ints');
    node.destroyClient(client);

    // OK to destroy a client multiple times
    node.destroyClient(client);
  });

  it('Destroy a non-existent service', function() {
    var node = rclnodejs.createNode('node5', '/non_existent');
    assertThrowsError(() => {
      node.destroyService(null);
    }, TypeError, 'Invalid argument', 'Trying to destroy an empty service!');

    const AddTwoInts = 'example_interfaces/srv/AddTwoInts';
    var service = node.createService(AddTwoInts, 'add_two_ints',
                                    (request, response) =>{});
    node.destroyService(service);

    // OK to destroy a service multiple times
    node.destroyService(service);
  });

  it('Destroy a non-existent timer', function() {
    var node = rclnodejs.createNode('node6', '/non_existent');
    assertThrowsError(() => {
      node.destroyTimer(null);
    }, TypeError, 'Invalid argument', 'Trying to destroy an empty timer!');

    var count = 0;
    var timer = node.createTimer(100, () => { count++; });
    node.destroyTimer(timer);

    // OK to destroy a timer multiple times
    node.destroyTimer(timer);
  });
});

describe('Fuzzing API calls testing', function() {
  this.timeout(60 * 1000);

  before(function() {
    return rclnodejs.init();
  });

  after(function() {
    rclnodejs.shutdown();
  });

  it('Unregistered message types', function() {
    var node = rclnodejs.createNode('node1', '/unregistered');
    const UnknownMsgType = 'std_msgs/msg/Foo';

    assertThrowsError(() => {
      node.createPublisher(UnknownMsgType, 'chatter');
    }, Error, 'does not exist', 'Unrecoginzed message types!');

    assertThrowsError(() => {
      node.createSubscription(UnknownMsgType, 'chatter', (msg) => {});
    }, Error, 'does not exist', 'Unrecoginzed message type!');

    node.destroy();
  });

  it('Unregistered service interfaces', function() {
    var node = rclnodejs.createNode('node2', '/unregistered');
    const UnknownInterface = 'example_interfaces/srv/Bar';

    assertThrowsError(() => {
      node.createClient(UnknownInterface, 'chatter');
    }, Error, 'does not exist', 'Unrecoginzed service interface!');

    assertThrowsError(() => {
      node.createService(UnknownInterface, 'chatter', (req, res) => {});
    }, Error, 'does not exist', 'Unrecoginzed service interface!');

    node.destroy();
  });

  it('Inconsistent message type for subscription', function() {
    var node = rclnodejs.createNode('node1', '/inconsistent');
    const RclString = 'std_msgs/msg/String';

    var publisher = node.createPublisher(RclString, 'chatter7');
    // assertThrowsError(() => {
    //   publisher.publish({a: 1});
    // }, TypeError, 'Illegal', `Illegal data for ${RclString}`);

    rclnodejs.spin(node);
    node.destroy();
  });

  it('Inconsistent request data for service', function(done) {
    var node = rclnodejs.createNode('node2', '/inconsistent');
    const AddTwoInts = 'example_interfaces/srv/AddTwoInts';

    var client = node.createClient(AddTwoInts, 'add_two_ints');
    var service = node.createService(AddTwoInts, 'add_two_ints', (request, response) => {
      // assert.deepStrictEqual(request.b, undefined);
      node.destroy();
      done();
    });
    client.sendRequest({a: 1}, (response) => {});
    rclnodejs.spin(node);
  });

  it('resources will be freed by shutdown', function() {
    var node = rclnodejs.createNode('node1', '/unhandled');
    const RclString = 'std_msgs/msg/String';
    const AddTwoInts = 'example_interfaces/srv/AddTwoInts';

    var publisher = node.createPublisher(RclString, 'chatter9');
    var subscription = node.createSubscription(RclString, 'chatter9', () => {});
    var client = node.createClient(AddTwoInts, 'add_two_ints');
    var service = node.createService(AddTwoInts, 'add_two_ints', (request, response) => {});
  });

  it('timer Creating with inconsistent type', function() {
    var node = rclnodejs.createNode('node3', '/inconsistent');
    const invalidParams = [
      ['100', () => {}],
      [100, null]
    ];
    invalidParams.forEach((param) => {
      assertThrowsError(() => {
        node.createTimer(param[0], param[1]);
      }, TypeError, 'Invalid argument', 'Failed to createTimer!');
    });

    node.destroy();
  });
});
