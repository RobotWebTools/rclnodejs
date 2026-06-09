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
import MessageIntrospector from '../lib/message_introspector.js';
import { TypeValidationError } from '../lib/errors.js';

describe('MessageIntrospector Tests', function () {
  before(async function () {
    try {
      await rclnodejs.init();
    } catch {}
  });

  after(function () {
    try {
      rclnodejs.shutdown();
    } catch {}
  });

  describe('Constructor', function () {
    it('should create an introspector for std_msgs/msg/String', function () {
      const introspector = new MessageIntrospector('std_msgs/msg/String');
      assert.strictEqual(introspector.typeName, 'std_msgs/msg/String');
    });

    it('should create an introspector for geometry_msgs/msg/Twist', function () {
      const introspector = new MessageIntrospector('geometry_msgs/msg/Twist');
      assert.strictEqual(introspector.typeName, 'geometry_msgs/msg/Twist');
    });

    it('should throw TypeValidationError for invalid typeName', function () {
      assert.throws(() => new MessageIntrospector(null), TypeValidationError);
      assert.throws(() => new MessageIntrospector(''), TypeValidationError);
      assert.throws(() => new MessageIntrospector(123), TypeValidationError);
    });

    it('should throw for unknown message type', function () {
      assert.throws(() => new MessageIntrospector('unknown/msg/Type'));
    });
  });

  describe('Properties', function () {
    it('should return correct fields for std_msgs/msg/String', function () {
      const introspector = new MessageIntrospector('std_msgs/msg/String');
      assert.deepStrictEqual(introspector.fields, ['data']);
    });

    it('should return correct fields for geometry_msgs/msg/Twist', function () {
      const introspector = new MessageIntrospector('geometry_msgs/msg/Twist');
      assert.deepStrictEqual(introspector.fields, ['linear', 'angular']);
    });

    it('should return correct fields for sensor_msgs/msg/JointState', function () {
      const introspector = new MessageIntrospector(
        'sensor_msgs/msg/JointState'
      );
      assert.deepStrictEqual(introspector.fields, [
        'header',
        'name',
        'position',
        'velocity',
        'effort',
      ]);
    });

    it('should have a valid schema', function () {
      const introspector = new MessageIntrospector('geometry_msgs/msg/Twist');
      assert.ok(introspector.schema);
      assert.ok(introspector.schema.fields);
      assert.strictEqual(introspector.schema.msgName, 'Twist');
    });

    it('should have a valid typeClass', function () {
      const introspector = new MessageIntrospector('geometry_msgs/msg/Twist');
      assert.ok(introspector.typeClass);
      assert.strictEqual(typeof introspector.typeClass, 'function');
    });
  });

  describe('defaults', function () {
    it('should return correct defaults for std_msgs/msg/String', function () {
      const introspector = new MessageIntrospector('std_msgs/msg/String');
      const defaults = introspector.defaults;
      assert.deepStrictEqual(defaults, { data: '' });
    });

    it('should return correct defaults for geometry_msgs/msg/Twist', function () {
      const introspector = new MessageIntrospector('geometry_msgs/msg/Twist');
      const defaults = introspector.defaults;
      assert.deepStrictEqual(defaults, {
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      });
    });

    it('should return a new object each time (no mutation)', function () {
      const introspector = new MessageIntrospector('geometry_msgs/msg/Twist');
      const defaults1 = introspector.defaults;
      const defaults2 = introspector.defaults;
      assert.notStrictEqual(defaults1, defaults2);

      defaults1.linear.x = 999;
      assert.strictEqual(defaults2.linear.x, 0);
    });
  });

  describe('Integration with rclnodejs', function () {
    it('should be accessible via rclnodejs.MessageIntrospector', function () {
      assert.strictEqual(rclnodejs.MessageIntrospector, MessageIntrospector);
    });

    it('should work for inspecting message structure', function () {
      const Twist = new rclnodejs.MessageIntrospector(
        'geometry_msgs/msg/Twist'
      );

      assert.deepStrictEqual(Twist.fields, ['linear', 'angular']);
      assert.deepStrictEqual(Twist.defaults, {
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      });
      assert.strictEqual(Twist.schema.msgName, 'Twist');
    });
  });
});
