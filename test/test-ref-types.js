// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

import assert from 'node:assert/strict';
import { inspect } from 'node:util';
import ref from '../third_party/ref-napi/index.js';
import createArrayType from '../third_party/ref-array-di/lib/array.js';
import createStructType from '../third_party/ref-struct-di/lib/struct.js';

const ArrayType = createArrayType(ref);
const StructType = createStructType(ref);

describe('Vendored ref types', function () {
  it('preserves custom Buffer inspection with native memory addresses', function () {
    const buffer = Buffer.from('ref');
    assert.equal(inspect(buffer), `<Buffer@0x${buffer.hexAddress()} 72 65 66>`);
  });

  it('preserves struct field offsets, alignment and backing-buffer access', function () {
    const Record = StructType({
      tag: ref.types.uint8,
      value: ref.types.uint32,
    });
    assert.equal(Record.fields.tag.offset, 0);
    assert.equal(Record.fields.value.offset, 4);
    assert.equal(Record.alignment, 4);
    assert.equal(Record.size, 8);

    const backing = Buffer.alloc(Record.size);
    const record = new Record(backing, { tag: 7, value: 42 });
    assert.equal(record.ref(), backing);
    assert.equal(ref.get(backing, 4, ref.types.uint32), 42);
    ref.set(backing, 4, 1234, ref.types.uint32);
    assert.deepEqual(record.toObject(), { tag: 7, value: 1234 });
    assert.throws(() => Record.defineProperty('extra', ref.types.int32));
    assert.throws(() => new Record(Buffer.alloc(Record.size - 1)));
  });

  it('preserves packed struct layout', function () {
    const Packed = StructType(
      { tag: ref.types.uint8, value: ref.types.uint32 },
      { packed: true }
    );
    assert.equal(Packed.fields.value.offset, 1);
    assert.equal(Packed.alignment, 1);
    assert.equal(Packed.size, 5);
    const record = new Packed({ tag: 2, value: 0x12345678 });
    assert.deepEqual(record.toObject(), { tag: 2, value: 0x12345678 });
  });

  it('preserves fixed arrays nested in structs and copy semantics', function () {
    const Pair = StructType({ x: ref.types.int16, y: ref.types.int16 });
    const Pairs = ArrayType(Pair, 2);
    const Record = StructType({
      tag: ref.types.uint8,
      pairs: Pairs,
      tail: ref.types.uint32,
    });
    assert.equal(Pairs.size, 8);
    assert.equal(Pairs.alignment, 2);
    assert.equal(Record.fields.pairs.offset, 2);
    assert.equal(Record.fields.tail.offset, 12);
    assert.equal(Record.size, 16);
    const record = new Record({
      tag: 1,
      pairs: [
        { x: 2, y: 3 },
        { x: 4, y: 5 },
      ],
      tail: 6,
    });
    const copy = Buffer.alloc(Record.size);
    Record.set(copy, 0, record);
    record.pairs[0].x = -7;
    assert.equal(Record.get(copy, 0).pairs[0].x, 2);
    assert.equal(
      ref.get(record.ref(), Record.fields.pairs.offset, ref.types.int16),
      -7
    );
  });

  it('preserves variable arrays, slices and pointer round trips', function () {
    const Ints = ArrayType(ref.types.int32);
    assert.equal(Ints.size, ref.sizeof.pointer);
    assert.equal(Ints.alignment, ref.alignof.pointer);
    const values = new Ints([1, 2, 3]);
    const slice = values.slice(1);
    slice[0] = 42;
    assert.deepEqual(values.toArray(), [1, 42, 3]);
    const dereferenced = ref.deref(values.ref());
    assert.deepEqual(dereferenced.toArray(), [1, 42, 3]);
    dereferenced[2] = -5;
    assert.equal(values[2], -5);
    assert.throws(() => new Ints());
    const Fixed = ArrayType(ref.types.int32, 2);
    assert.equal(new Fixed().length, 2);
    assert.throws(() => new Fixed([1]));
    assert.throws(() => new Fixed(Buffer.alloc(4)));
  });

  it('accepts PointerBuffer-backed arrays without copying', function () {
    const Ints = ArrayType(ref.types.int32, 2);
    const backing = Buffer.alloc(Ints.size);
    const pointer = ref.reinterpret(backing, backing.length);
    assert.ok(ref.PointerBuffer.isPointerBuffer(pointer));
    const values = new Ints(pointer);
    assert.equal(values.buffer, pointer);
    values[0] = 42;
    values[1] = -7;
    assert.equal(ref.get(backing, 0, ref.types.int32), 42);
    ref.set(backing, 4, 123, ref.types.int32);
    assert.deepEqual(values.toArray(), [42, 123]);
  });

  it('accepts PointerBuffer-backed structs without copying', function () {
    const Pair = StructType({ x: ref.types.int32, y: ref.types.int32 });
    const backing = Buffer.alloc(Pair.size);
    const pointer = ref.reinterpret(backing, backing.length);
    assert.ok(ref.PointerBuffer.isPointerBuffer(pointer));
    const pair = new Pair(pointer, { x: 12, y: -34 });
    assert.equal(pair.ref(), pointer);
    assert.equal(ref.get(backing, 0, ref.types.int32), 12);
    ref.set(backing, 4, 56, ref.types.int32);
    assert.deepEqual(pair.toObject(), { x: 12, y: 56 });
  });
});
