// Copyright (c) 2025 Mahmoud Alghalayini. All rights reserved.
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
const { take, map, filter, toArray } = require('rxjs');

describe('rclnodejs observable subscription test suite', function () {
  this.timeout(60 * 1000);

  beforeEach(function () {
    return rclnodejs.init();
  });

  afterEach(function () {
    rclnodejs.shutdown();
  });

  it('createObservableSubscription returns an ObservableSubscription', function () {
    const node = new rclnodejs.Node('test_node');
    const String = 'std_msgs/msg/String';

    const obsSub = node.createObservableSubscription(String, 'test_topic');

    assert.ok(obsSub, 'ObservableSubscription should be created');
    assert.ok(obsSub.observable, 'Should have an observable property');
    assert.ok(obsSub.subscription, 'Should have a subscription property');
    assert.ok(obsSub.topic.includes('test_topic'));
    assert.strictEqual(obsSub.isDestroyed, false);

    node.destroy();
  });

  it('ObservableSubscription exports the class', function () {
    assert.ok(
      rclnodejs.ObservableSubscription,
      'ObservableSubscription should be exported'
    );
  });

  it('Observable receives messages from publisher', function (done) {
    const node = new rclnodejs.Node('test_node');
    const String = 'std_msgs/msg/String';

    const publisher = node.createPublisher(String, 'observable_test_topic');
    const obsSub = node.createObservableSubscription(
      String,
      'observable_test_topic'
    );

    const receivedMessages = [];
    let interval;

    obsSub.observable.pipe(take(3)).subscribe({
      next: (msg) => {
        receivedMessages.push(msg.data);
      },
      complete: () => {
        clearInterval(interval);
        assert.strictEqual(receivedMessages.length, 3);
        assert.deepStrictEqual(receivedMessages, [
          'message_0',
          'message_1',
          'message_2',
        ]);
        node.destroy();
        done();
      },
      error: (err) => {
        clearInterval(interval);
        node.destroy();
        done(err);
      },
    });

    node.spin();

    let count = 0;
    interval = setInterval(() => {
      publisher.publish({ data: `message_${count}` });
      count++;
      if (count >= 10) {
        clearInterval(interval);
      }
    }, 50);
  });

  it('Observable can use RxJS operators', function (done) {
    const node = new rclnodejs.Node('test_node');
    const Int32 = 'std_msgs/msg/Int32';

    const publisher = node.createPublisher(Int32, 'rxjs_operators_topic');
    const obsSub = node.createObservableSubscription(
      Int32,
      'rxjs_operators_topic'
    );

    let interval;

    obsSub.observable
      .pipe(
        take(5),
        map((msg) => msg.data),
        filter((data) => data % 2 === 0),
        toArray()
      )
      .subscribe({
        next: (results) => {
          // From 0,1,2,3,4 we filter even: 0,2,4
          assert.deepStrictEqual(results, [0, 2, 4]);
        },
        complete: () => {
          clearInterval(interval);
          node.destroy();
          done();
        },
        error: (err) => {
          clearInterval(interval);
          node.destroy();
          done(err);
        },
      });

    node.spin();

    let count = 0;
    interval = setInterval(() => {
      publisher.publish({ data: count });
      count++;
      if (count >= 10) {
        clearInterval(interval);
      }
    }, 50);
  });

  it('complete() stops the observable', function (done) {
    const node = new rclnodejs.Node('test_node');
    const String = 'std_msgs/msg/String';

    const obsSub = node.createObservableSubscription(
      String,
      'complete_test_topic'
    );

    let completed = false;

    obsSub.observable.subscribe({
      complete: () => {
        completed = true;
      },
    });

    obsSub.complete();

    assert.strictEqual(obsSub.isDestroyed, true);
    assert.strictEqual(completed, true);

    node.destroy();
    done();
  });

  it('destroy() is an alias for complete()', function () {
    const node = new rclnodejs.Node('test_node');
    const String = 'std_msgs/msg/String';

    const obsSub = node.createObservableSubscription(
      String,
      'destroy_test_topic'
    );

    obsSub.destroy();

    assert.strictEqual(obsSub.isDestroyed, true);

    node.destroy();
  });

  it('can be used with options', function () {
    const node = new rclnodejs.Node('test_node');
    const String = 'std_msgs/msg/String';

    const obsSub = node.createObservableSubscription(
      String,
      'options_test_topic',
      {
        qos: rclnodejs.QoS.profileDefault,
        enableTypedArray: false,
      }
    );

    assert.ok(obsSub, 'ObservableSubscription should be created with options');
    assert.ok(obsSub.observable, 'Should have an observable property');

    node.destroy();
  });

  it('underlying subscription can be destroyed via node', function () {
    const node = new rclnodejs.Node('test_node');
    const String = 'std_msgs/msg/String';

    const obsSub = node.createObservableSubscription(
      String,
      'destroy_sub_test_topic'
    );

    node.destroySubscription(obsSub.subscription);

    node.destroy();
  });
});
