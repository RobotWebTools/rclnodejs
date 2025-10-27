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

describe('Serialization Modes Tests', function () {
  let node;

  this.timeout(60 * 1000);

  before(async function () {
    await rclnodejs.init();
    node = rclnodejs.createNode('test_serialization_modes');
  });

  after(function () {
    rclnodejs.shutdown();
  });

  it('Test toJSONSafe utility function', function () {
    const float64Array = new Float64Array([1.1, 2.2, 3.3]);
    const jsonSafe = rclnodejs.toJSONSafe(float64Array);

    assert.ok(Array.isArray(jsonSafe));
    assert.strictEqual(jsonSafe.length, 3);
    assert.strictEqual(jsonSafe[0], 1.1);
    assert.strictEqual(jsonSafe[1], 2.2);
    assert.strictEqual(jsonSafe[2], 3.3);
  });

  it('Test toJSONString utility function', function () {
    const testObject = {
      typedArray: new Float64Array([1.5, 2.5]),
      bigIntValue: BigInt(123456789),
      infinity: Infinity,
      negInfinity: -Infinity,
      nanValue: NaN,
      regular: 'string',
    };

    const jsonString = rclnodejs.toJSONString(testObject);
    const parsed = JSON.parse(jsonString);

    assert.ok(Array.isArray(parsed.typedArray));
    assert.strictEqual(parsed.typedArray[0], 1.5);

    assert.strictEqual(parsed.bigIntValue, '123456789n');

    assert.strictEqual(parsed.infinity, 'Infinity');
    assert.strictEqual(parsed.negInfinity, '-Infinity');
    assert.strictEqual(parsed.nanValue, 'NaN');

    assert.strictEqual(parsed.regular, 'string');
  });

  it('Test nested object conversion', function () {
    const nestedObject = {
      level1: {
        level2: {
          typedArray: new Int32Array([10, 20, 30]),
          regularArray: [1, 2, 3],
        },
      },
    };

    const jsonSafe = rclnodejs.toJSONSafe(nestedObject);

    assert.ok(Array.isArray(jsonSafe.level1.level2.typedArray));
    assert.strictEqual(jsonSafe.level1.level2.typedArray[0], 10);

    assert.ok(Array.isArray(jsonSafe.level1.level2.regularArray));
    assert.strictEqual(jsonSafe.level1.level2.regularArray[0], 1);
  });

  it('Test serializationMode option validation', function () {
    assert.doesNotThrow(() => {
      node._validateOptions({ serializationMode: 'default' });
      node._validateOptions({ serializationMode: 'plain' });
      node._validateOptions({ serializationMode: 'json' });
    });

    assert.throws(() => {
      node._validateOptions({ serializationMode: 'invalid' });
    }, /Invalid serializationMode/);
  });

  it('Test subscription with different serialization modes', function (done) {
    let typedSubscription, plainSubscription, jsonSubscription;

    try {
      typedSubscription = node.createSubscription(
        'std_msgs/msg/String',
        '/test_topic_typed',
        { serializationMode: 'default' },
        () => {}
      );

      plainSubscription = node.createSubscription(
        'std_msgs/msg/String',
        '/test_topic_plain',
        { serializationMode: 'plain' },
        () => {}
      );

      jsonSubscription = node.createSubscription(
        'std_msgs/msg/String',
        '/test_topic_json',
        { serializationMode: 'json' },
        () => {}
      );

      assert.strictEqual(typedSubscription.serializationMode, 'default');
      assert.strictEqual(plainSubscription.serializationMode, 'plain');
      assert.strictEqual(jsonSubscription.serializationMode, 'json');

      done();
    } catch (error) {
      done(error);
    }
  });

  it('Test default serializationMode', function () {
    const subscription = node.createSubscription(
      'std_msgs/msg/String',
      '/test_topic_default',
      {},
      () => {}
    );

    assert.strictEqual(subscription.serializationMode, 'default');
  });
});
