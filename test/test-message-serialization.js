'use strict';

const assert = require('assert');
const {
  isTypedArray,
  needsJSONConversion,
  toPlainArrays,
  toJSONSafe,
  toJSONString,
  applySerializationMode,
  isValidSerializationMode,
} = require('../lib/message_serialization.js');
const rclnodejs = require('../index.js');

describe('Message Serialization coverage testing', function () {
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
    assert.strictEqual(output.undef, null);
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
});
