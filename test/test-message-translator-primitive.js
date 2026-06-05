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

import rclnodejs from '../index.js';
import deepEqual from 'deep-equal';
import * as arrayGen from './array_generator.js';
import { isTypedArray } from './utils.js';

// In IDL mode, .msg 'char' type is mapped to 'uint8' (they are identical in IDL spec)
const useIDL = process.argv.includes('--idl');

describe('Rclnodejs message translation: primitive types', function () {
  this.timeout(60 * 1000);

  before(function () {
    return rclnodejs.init();
  });

  after(function () {
    rclnodejs.shutdown();
  });

  [
    { type: 'Bool', values: [true, false] },
    { type: 'Byte', values: [0, 1, 2, 3, 255] },
    {
      type: 'Char',
      values: useIDL
        ? [0, 1, 2, 3, 127, 255]
        : [-128, -127, -2, -1, 0, 1, 2, 3, 127],
    },
    { type: 'Float32', values: [-5, 0, 1.25, 89.75, 72.5, 3.14e5] },
    { type: 'Float64', values: [-5, 0, 1.25, 89.75, 72.5, 3.14159265358e8] },
    { type: 'Int16', values: [-32768, -2, -1, 0, 1, 2, 3, 32767] },
    {
      type: 'Int32',
      values: [-2147483648, -32768, -2, -1, 0, 1, 2, 3, 32767, 2147483647],
    },
    {
      type: 'Int64',
      values: [
        -32768n,
        -2n,
        -1n,
        0n,
        1n,
        2n,
        3n,
        32767n,
        2147483648n,
        BigInt(Number.MAX_SAFE_INTEGER) + 1n,
      ],
    },
    { type: 'Int8', values: [-128, -127, -2, -1, 0, 1, 2, 3, 127] },
    { type: 'String', values: ['', 'A String', ' ', '<>', '©'] },
    { type: 'UInt16', values: [0, 1, 2, 3, 32767, 65535] },
    {
      type: 'UInt32',
      values: [0, 1, 2, 3, 32767, 65535, 2147483648, 4294967295],
    },
    {
      type: 'UInt64',
      values: [
        0n,
        1n,
        2n,
        3n,
        32767n,
        65535n,
        2147483648n,
        4294967295n,
        BigInt(Number.MAX_SAFE_INTEGER) + 1n,
      ],
    },
    { type: 'UInt8', values: [0, 1, 2, 3, 127, 255] },
  ].forEach((testData) => {
    const topic = testData.topic || 'topic' + testData.type + 'Shortcut';
    testData.values.forEach((v, i) => {
      it(
        'Test translation of ' + testData.type + ' msg, value ' + v,
        function () {
          const node = rclnodejs.createNode('test_message_translation_node');
          const MessageType = 'std_msgs/msg/' + testData.type;
          const publisher = node.createPublisher(MessageType, topic);
          return new Promise((resolve, reject) => {
            const sub = node.createSubscription(MessageType, topic, (value) => {
              // For primitive types, msgs are defined as a single `.data` field
              if (value.data === v) {
                node.destroy();
                resolve();
              } else {
                node.destroy();
                reject(
                  'case ' + i + '. Expected: ' + v + ', Got: ' + value.data
                );
              }
            });
            publisher.publish(v); // Short-cut form of publishing primitive types
            rclnodejs.spin(node);
          });
        }
      );

      it(
        'Test translation of ' + testData.type + ' msg, value ' + v + '(.data)',
        function () {
          const node = rclnodejs.createNode('test_message_translation_node');
          const MessageType = 'std_msgs/msg/' + testData.type;
          const publisher = node.createPublisher(MessageType, topic);
          return new Promise((resolve, reject) => {
            const sub = node.createSubscription(MessageType, topic, (value) => {
              // For primitive types, msgs are defined as a single `.data` field
              if (value.data === v) {
                node.destroy();
                resolve();
              } else {
                node.destroy();
                reject(
                  'case ' + i + '. Expected: ' + v + ', Got: ' + value.data
                );
              }
            });
            publisher.publish({ data: v }); // Ensure the original form of the message can be used
            rclnodejs.spin(node);
          });
        }
      );
    });
  });
});

describe('Rclnodejs message translation: primitive types array', function () {
  this.timeout(60 * 1000);

  before(function () {
    return rclnodejs.init();
  });

  after(function () {
    rclnodejs.shutdown();
  });

  [
    { type: 'ByteMultiArray', values: [0, 1, 2, 3, 255] },
    {
      type: 'Float32MultiArray',
      values: [
        -5,
        0,
        1.25,
        89.75,
        72.5,
        3.141592e8,
        Number.POSITIVE_INFINITY,
        Number.NEGATIVE_INFINITY,
      ],
    },
    {
      type: 'Float64MultiArray',
      values: [
        -5,
        0,
        1.25,
        89.75,
        72.5,
        3.141592e8,
        Number.POSITIVE_INFINITY,
        Number.NEGATIVE_INFINITY,
      ],
    },
    { type: 'Int8MultiArray', values: [-128, -127, -2, -1, 0, 1, 2, 3, 127] },
    { type: 'Int16MultiArray', values: [-32768, -2, -1, 0, 1, 2, 3, 32767] },
    {
      type: 'Int32MultiArray',
      values: [
        -2147483648, -2147483647, -32768, -2, -1, 0, 1, 2, 3, 32767, 2147483647,
      ],
    },
    {
      type: 'Int64MultiArray',
      values: [
        BigInt(-Number.MAX_SAFE_INTEGER),
        -32768n,
        -2n,
        -1n,
        0n,
        1n,
        2n,
        3n,
        32767n,
        BigInt(Number.MAX_SAFE_INTEGER),
      ],
    },
    { type: 'UInt8MultiArray', values: [0, 1, 2, 3, 127, 255] },
    { type: 'UInt16MultiArray', values: [0, 1, 2, 3, 32767, 65535] },
    {
      type: 'UInt32MultiArray',
      values: [0, 1, 2, 3, 32767, 65535, 4294967294, 4294967295],
    },
    {
      type: 'UInt64MultiArray',
      values: [0n, 1n, 2n, 3n, 32767n, 65535n, BigInt(Number.MAX_SAFE_INTEGER)],
    },
  ].forEach((testData) => {
    const topic = testData.topic || 'topic' + testData.type;
    it(
      'Test translation of ' + testData.type + ' msg, value ' + testData.values,
      function () {
        const node = rclnodejs.createNode('test_message_translation_node');
        const MessageType = 'std_msgs/msg/' + testData.type;
        const publisher = node.createPublisher(MessageType, topic);
        return new Promise((resolve, reject) => {
          const sub = node.createSubscription(MessageType, topic, (value) => {
            // For primitive types, msgs are defined as a single `.data` field
            if (
              (isTypedArray(value.data) &&
                deepEqual(Array.from(value.data), testData.values)) ||
              deepEqual(value.data, testData.values)
            ) {
              node.destroy();
              resolve();
            } else {
              node.destroy();
              reject('Expected: ' + testData.values + ', Got: ' + value.data);
            }
          });
          publisher.publish({
            layout: {
              dim: [{ label: 'length', size: 0, stride: 0 }],
              data_offset: 0,
            },
            data: testData.values,
          });
          rclnodejs.spin(node);
        });
      }
    );
  });
});

describe('Rclnodejs message translation: TypedArray large data', function () {
  this.timeout(60 * 1000);

  before(function () {
    return rclnodejs.init();
  });

  after(function () {
    rclnodejs.shutdown();
  });

  const arrayLength = 100 * 1000;
  [
    {
      type: 'ByteMultiArray',
      values: arrayGen.generateValues(
        Uint8Array,
        arrayLength,
        256,
        arrayGen.positive,
        Math.floor
      ),
    },
    {
      type: 'Float32MultiArray',
      values: arrayGen.generateValues(
        Float32Array,
        arrayLength,
        100000000,
        arrayGen.negative,
        arrayGen.noRound
      ),
    },
    {
      type: 'Float32MultiArray',
      values: arrayGen.generateValues(
        Float32Array,
        arrayLength,
        10000,
        arrayGen.negative,
        arrayGen.noRound
      ),
    },
    {
      type: 'Float64MultiArray',
      values: arrayGen.generateValues(
        Float64Array,
        arrayLength,
        Number.MAX_VALUE,
        arrayGen.negative,
        arrayGen.noRound
      ),
    },
    {
      type: 'Float64MultiArray',
      values: arrayGen.generateValues(
        Float64Array,
        arrayLength,
        10000,
        arrayGen.negative,
        arrayGen.noRound
      ),
    },
    {
      type: 'Int8MultiArray',
      values: arrayGen.generateValues(
        Int8Array,
        arrayLength,
        128,
        arrayGen.negative,
        Math.floor
      ),
    },
    {
      type: 'Int16MultiArray',
      values: arrayGen.generateValues(
        Int16Array,
        arrayLength,
        32768,
        arrayGen.negative,
        Math.floor
      ),
    },
    {
      type: 'Int32MultiArray',
      values: arrayGen.generateValues(
        Int32Array,
        arrayLength,
        2147483648,
        arrayGen.negative,
        Math.floor
      ),
    },
    {
      type: 'UInt8MultiArray',
      values: arrayGen.generateValues(
        Uint8Array,
        arrayLength,
        256,
        arrayGen.positive,
        Math.floor
      ),
    },
    {
      type: 'UInt16MultiArray',
      values: arrayGen.generateValues(
        Uint16Array,
        arrayLength,
        65536,
        arrayGen.positive,
        Math.floor
      ),
    },
    {
      type: 'UInt32MultiArray',
      values: arrayGen.generateValues(
        Uint32Array,
        arrayLength,
        4294967296,
        arrayGen.positive,
        Math.floor
      ),
    },

    {
      type: 'ByteMultiArray',
      values: arrayGen.generateValues(
        Array,
        arrayLength,
        256,
        arrayGen.positive,
        Math.floor
      ),
    },
    // Note: According to IEEE 754, float32 has 6 significant decimal digits, skip float32 for now
    // {type: 'Float32MultiArray', values: arrayGen.generateValues(Array, arrayLength, 100000000, arrayGen.negative, arrayGen.noRound)},
    // {type: 'Float32MultiArray', values: arrayGen.generateValues(Array, arrayLength, 10000, arrayGen.negative, arrayGen.noRound)},
    {
      type: 'Float64MultiArray',
      values: arrayGen.generateValues(
        Array,
        arrayLength,
        Number.MAX_VALUE,
        arrayGen.negative,
        arrayGen.noRound
      ),
    },
    {
      type: 'Float64MultiArray',
      values: arrayGen.generateValues(
        Array,
        arrayLength,
        10000,
        arrayGen.negative,
        arrayGen.noRound
      ),
    },
    {
      type: 'Int8MultiArray',
      values: arrayGen.generateValues(
        Array,
        arrayLength,
        128,
        arrayGen.negative,
        Math.floor
      ),
    },
    {
      type: 'Int16MultiArray',
      values: arrayGen.generateValues(
        Array,
        arrayLength,
        32768,
        arrayGen.negative,
        Math.floor
      ),
    },
    {
      type: 'Int32MultiArray',
      values: arrayGen.generateValues(
        Array,
        arrayLength,
        2147483648,
        arrayGen.negative,
        Math.floor
      ),
    },
    {
      type: 'UInt8MultiArray',
      values: arrayGen.generateValues(
        Array,
        arrayLength,
        256,
        arrayGen.positive,
        Math.floor
      ),
    },
    {
      type: 'UInt16MultiArray',
      values: arrayGen.generateValues(
        Array,
        arrayLength,
        65536,
        arrayGen.positive,
        Math.floor
      ),
    },
    {
      type: 'UInt32MultiArray',
      values: arrayGen.generateValues(
        Array,
        arrayLength,
        4294967296,
        arrayGen.positive,
        Math.floor
      ),
    },
  ].forEach((testData) => {
    const topic = testData.topic || 'topic' + testData.type;
    it(
      'Test translation of ' +
        testData.type +
        ' msg, number of values ' +
        testData.values.length,
      function () {
        const node = rclnodejs.createNode('test_message_translation_node');
        const MessageType = 'std_msgs/msg/' + testData.type;
        const publisher = node.createPublisher(MessageType, topic);
        return new Promise((resolve, reject) => {
          const sub = node.createSubscription(MessageType, topic, (value) => {
            // For primitive types, msgs are defined as a single `.data` field
            if (
              (isTypedArray(value.data) &&
                deepEqual(Array.from(value.data), testData.values)) ||
              deepEqual(value.data, testData.values)
            ) {
              node.destroy();
              resolve();
            } else {
              node.destroy();
              reject(
                'Expected: ' +
                  testData.values +
                  ',                              Got: ' +
                  value.data
              );
            }
          });
          publisher.publish({
            layout: {
              dim: [{ label: 'length', size: 0, stride: 0 }],
              data_offset: 0,
            },
            data: testData.values,
          });
          rclnodejs.spin(node);
        });
      }
    );
  });
});
