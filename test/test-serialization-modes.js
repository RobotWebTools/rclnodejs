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

import assert from 'assert';
import rclnodejs from '../index.js';
import {
  isTypedArray,
  needsJSONConversion,
  toPlainArrays,
  toJSONSafe,
  toJSONString,
  applySerializationMode,
  isValidSerializationMode,
  reviveBigInts,
} from '../lib/message_serialization.js';

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

describe('Message Serialization Unit Tests', function () {
  it('isTypedArray identifies TypedArrays', function () {
    assert.strictEqual(isTypedArray(new Uint8Array(1)), true);
    assert.strictEqual(isTypedArray(new Int32Array(1)), true);
    assert.strictEqual(isTypedArray(new Float64Array(1)), true);
    assert.strictEqual(isTypedArray([]), false);
    assert.strictEqual(isTypedArray(new DataView(new ArrayBuffer(8))), false);
    assert.strictEqual(isTypedArray(null), false);
  });

  it('needsJSONConversion identifies special types', function () {
    assert.strictEqual(needsJSONConversion(10n), true);
    assert.strictEqual(
      needsJSONConversion(() => {}),
      true
    );
    assert.strictEqual(needsJSONConversion(undefined), true);
    assert.strictEqual(needsJSONConversion(Infinity), true);
    assert.strictEqual(needsJSONConversion(-Infinity), true);
    assert.strictEqual(needsJSONConversion(NaN), true);
    assert.strictEqual(needsJSONConversion(123), false);
    assert.strictEqual(needsJSONConversion('string'), false);
    assert.strictEqual(needsJSONConversion(null), false);
  });

  it('toPlainArrays converts recursively', function () {
    const input = {
      a: new Uint8Array([1, 2]),
      b: {
        c: new Float32Array([1.1, 2.2]),
        d: [new Int8Array([3])],
      },
      e: null,
      f: undefined, // Should handle
    };

    // Note: Float32Array precision might cause exact equality issues if we compare strict deep equal with floats,
    // but here we just check structure and array type.
    const output = toPlainArrays(input);

    assert.ok(Array.isArray(output.a));
    assert.strictEqual(output.a[0], 1);

    assert.ok(Array.isArray(output.b.c));
    assert.ok(output.b.c.length === 2);

    assert.ok(Array.isArray(output.b.d[0]));
    assert.strictEqual(output.b.d[0][0], 3);

    assert.strictEqual(output.e, null);
  });

  it('toJSONSafe converts special types', function () {
    const input = {
      big: 123n,
      inf: Infinity,
      ninf: -Infinity,
      nan: NaN,
      undef: undefined,
      func: () => {},
      nested: {
        arr: [10n],
      },
      typed: new Uint8Array([1, 2]),
    };

    const output = toJSONSafe(input);

    assert.strictEqual(output.big, '123n');
    assert.strictEqual(output.inf, 'Infinity');
    assert.strictEqual(output.ninf, '-Infinity');
    assert.strictEqual(output.nan, 'NaN');
    assert.strictEqual(output.undef, undefined);
    assert.strictEqual(output.func, '[Function]');
    assert.strictEqual(output.nested.arr[0], '10n');

    // TypedArray in toJSONSafe is mapped to array and then items processed
    assert.ok(Array.isArray(output.typed));
    assert.strictEqual(output.typed[0], 1);
  });

  it('toJSONString produces string', function () {
    const input = { data: 123n };
    const str = toJSONString(input);
    assert.strictEqual(JSON.parse(str).data, '123n');
  });

  it('applySerializationMode works for all modes', function () {
    const input = { data: new Uint8Array([1]) };

    // default
    assert.strictEqual(applySerializationMode(input, 'default'), input);

    // plain
    const plain = applySerializationMode(input, 'plain');
    assert.ok(Array.isArray(plain.data));

    // json
    const json = applySerializationMode({ data: 10n }, 'json');
    assert.strictEqual(json.data, '10n');

    // invalid
    assert.throws(() => {
      applySerializationMode(input, 'invalid');
    }, /Invalid serializationMode/);
  });

  it('isValidSerializationMode', function () {
    assert.strictEqual(isValidSerializationMode('default'), true);
    assert.strictEqual(isValidSerializationMode('plain'), true);
    assert.strictEqual(isValidSerializationMode('json'), true);
    assert.strictEqual(isValidSerializationMode('other'), false);
  });

  describe('reviveBigInts', function () {
    it('rehydrates a top-level "Nn" string into a bigint', function () {
      assert.strictEqual(reviveBigInts('42n'), 42n);
      assert.strictEqual(reviveBigInts('-7n'), -7n);
      assert.strictEqual(reviveBigInts('0n'), 0n);
    });

    it('leaves non-BigInt strings untouched', function () {
      assert.strictEqual(reviveBigInts('hello'), 'hello');
      assert.strictEqual(reviveBigInts('42'), '42');
      assert.strictEqual(reviveBigInts('n'), 'n');
      assert.strictEqual(reviveBigInts(''), '');
    });

    it('passes through other primitives unchanged', function () {
      assert.strictEqual(reviveBigInts(42), 42);
      assert.strictEqual(reviveBigInts(true), true);
      assert.strictEqual(reviveBigInts(null), null);
      assert.strictEqual(reviveBigInts(undefined), undefined);
    });

    it('rehydrates nested arrays and objects', function () {
      const input = {
        sum: '42n',
        items: ['1n', '2n', 'plain'],
        nested: { count: '7n', name: 'foo' },
      };
      const out = reviveBigInts(input);
      assert.strictEqual(out.sum, 42n);
      assert.deepStrictEqual(out.items, [1n, 2n, 'plain']);
      assert.strictEqual(out.nested.count, 7n);
      assert.strictEqual(out.nested.name, 'foo');
    });

    it('round-trips toJSONSafe → reviveBigInts for bigint fields', function () {
      const original = { a: 9007199254740993n, b: { c: -1n, d: [5n, 6n] } };
      const wire = JSON.parse(JSON.stringify(toJSONSafe(original)));
      const revived = reviveBigInts(wire);
      assert.strictEqual(revived.a, original.a);
      assert.strictEqual(revived.b.c, original.b.c);
      assert.deepStrictEqual(revived.b.d, original.b.d);
    });

    it('refuses to be a prototype-pollution vector', function () {
      const polluted = JSON.parse(
        '{"__proto__":{"polluted":true},"constructor":{"prototype":{"polluted":true}},"prototype":{"polluted":true},"safe":"1n"}'
      );
      const out = reviveBigInts(polluted);
      // Object.prototype must remain untouched.
      assert.strictEqual({}.polluted, undefined);
      // Forbidden keys are dropped from the result.
      assert.strictEqual(
        Object.prototype.hasOwnProperty.call(out, '__proto__'),
        false
      );
      assert.strictEqual(
        Object.prototype.hasOwnProperty.call(out, 'constructor'),
        false
      );
      assert.strictEqual(
        Object.prototype.hasOwnProperty.call(out, 'prototype'),
        false
      );
      // Legitimate fields are still revived.
      assert.strictEqual(out.safe, 1n);
      // And the revived object has a null prototype, so further proto access is inert.
      assert.strictEqual(Object.getPrototypeOf(out), null);
    });
  });
});
