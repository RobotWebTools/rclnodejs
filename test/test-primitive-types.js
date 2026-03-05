// Copyright (c) 2026, The Robot Web Tools Contributors
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
const primitiveTypes = require('../rosidl_gen/primitive_types.js');

describe('rosidl_gen primitive_types', function () {
  describe('initString(str, own=true)', function () {
    it('should throw TypeError when str is a number', function () {
      assert.throws(
        () => primitiveTypes.initString(123, true),
        TypeError,
        'should provide a Node Buffer'
      );
    });

    it('should throw TypeError when str is a plain string', function () {
      assert.throws(
        () => primitiveTypes.initString('hello', true),
        TypeError,
        'should provide a Node Buffer'
      );
    });

    it('should throw TypeError when str is null', function () {
      assert.throws(
        () => primitiveTypes.initString(null, true),
        TypeError,
        'should provide a Node Buffer'
      );
    });

    it('should throw TypeError when str is undefined', function () {
      assert.throws(
        () => primitiveTypes.initString(undefined, true),
        TypeError,
        'should provide a Node Buffer'
      );
    });
  });

  describe('initString(str, own=false)', function () {
    it('should throw TypeError when str is a number', function () {
      assert.throws(
        () => primitiveTypes.initString(123),
        TypeError,
        'should provide a type of StringRefStruct'
      );
    });

    it('should throw TypeError when str is a plain string', function () {
      assert.throws(
        () => primitiveTypes.initString('hello'),
        TypeError,
        'should provide a type of StringRefStruct'
      );
    });

    it('should throw TypeError when str is a Buffer', function () {
      assert.throws(
        () => primitiveTypes.initString(Buffer.alloc(10)),
        TypeError,
        'should provide a type of StringRefStruct'
      );
    });

    it('should throw TypeError when str is null', function () {
      assert.throws(
        () => primitiveTypes.initString(null),
        TypeError,
        'should provide a type of StringRefStruct'
      );
    });

    it('should initialize a valid StringRefStruct', function () {
      const str = new primitiveTypes.string();
      primitiveTypes.initString(str);
      assert.strictEqual(str.size, 0);
      assert.strictEqual(str.capacity, 1);
    });
  });
});
