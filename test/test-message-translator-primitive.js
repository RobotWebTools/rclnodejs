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

const rclnodejs = require('../index.js');
const deepEqual = require('deep-equal');
const arrayGen = require('./array_generator.js');

/* eslint-disable camelcase */
/* eslint-disable indent */

describe('Rclnodejs message translation: primitive types', function() {
  this.timeout(60 * 1000);

  before(function() {
    return rclnodejs.init();
  });

  after(function() {
    rclnodejs.shutdown();
  });

  [
    {type: 'Bool',    values: [true, false]},
    {type: 'Byte',    values: [0, 1, 2, 3, 255]},
    {type: 'Char',    values: [-128, -127, -2, -1, 0, 1, 2, 3, 127]},
    {type: 'Float32', values: [-5, 0, 1.25, 89.75, 72.50, 3.14e5]},
    {type: 'Float64', values: [-5, 0, 1.25, 89.75, 72.50, 3.14159265358e8]},
    {type: 'Int16',   values: [-32768, -2, -1, 0, 1, 2, 3, 32767]},
    {type: 'Int32',   values: [-2147483648, -32768, -2, -1, 0, 1, 2, 3, 32767, 2147483647]},
    {type: 'Int64',   values: [-32768, -2, -1, 0, 1, 2, 3, 32767, 2147483648, 4294967295, Number.MAX_SAFE_INTEGER]},
    {type: 'Int8',    values: [-128, -127, -2, -1, 0, 1, 2, 3, 127]},
    {type: 'String',  values: ['', 'A String', ' ', '<>', '©']},
    {type: 'UInt16',  values: [0, 1, 2, 3, 32767, 65535]},
    {type: 'UInt32',  values: [0, 1, 2, 3, 32767, 65535, 2147483648, 4294967295]},
    {type: 'UInt64',  values: [0, 1, 2, 3, 32767, 65535, 2147483648, 4294967295, Number.MAX_SAFE_INTEGER]},
    {type: 'UInt8',   values: [0, 1, 2, 3, 127, 255]},
  ].forEach((testData) => {
    const topic = testData.topic || 'topic' + testData.type + 'Shortcut';
    testData.values.forEach((v, i) => {
      it('Test translation of ' + testData.type + ' msg, value ' + v, function() {
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
              reject('case ' + i + '. Expected: ' + v + ', Got: ' + value.data);
            }
          });
          publisher.publish(v);  // Short-cut form of publishing primitive types
          rclnodejs.spin(node);
        });
      });

      it('Test translation of ' + testData.type + ' msg, value ' + v + '(.data)', function() {
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
              reject('case ' + i + '. Expected: ' + v + ', Got: ' + value.data);
            }
          });
          publisher.publish({data: v});  // Ensure the original form of the message can be used
          rclnodejs.spin(node);
        });
      });

    });
  });
});

describe('Rclnodejs message translation: primitive types array', function() {
  this.timeout(60 * 1000);

  before(function() {
    return rclnodejs.init();
  });

  after(function() {
    rclnodejs.shutdown();
  });

  [
    {type: 'ByteMultiArray',    values: [0, 1, 2, 3, 255]},
    {type: 'Float32MultiArray', values: [-5, 0, 1.25, 89.75, 72.50, 3.141592e8,
                                         Number.POSITIVE_INFINITY, Number.NEGATIVE_INFINITY]},
    {type: 'Float64MultiArray', values: [-5, 0, 1.25, 89.75, 72.50, 3.141592e8,
                                         Number.POSITIVE_INFINITY, Number.NEGATIVE_INFINITY]},
    {type: 'Int8MultiArray',    values: [-128, -127, -2, -1, 0, 1, 2, 3, 127]},
    {type: 'Int16MultiArray',   values: [-32768, -2, -1, 0, 1, 2, 3, 32767]},
    {type: 'Int32MultiArray',   values: [-2147483648, -2147483647, -32768, -2, -1, 0, 1, 2, 3, 32767, 2147483647]},
    {type: 'Int64MultiArray',   values: [-Number.MAX_SAFE_INTEGER, -32768, -2, -1, 0, 1, 2, 3, 32767,
                                         Number.MAX_SAFE_INTEGER]},
    {type: 'UInt8MultiArray',   values: [0, 1, 2, 3, 127, 255]},
    {type: 'UInt16MultiArray',  values: [0, 1, 2, 3, 32767, 65535]},
    {type: 'UInt32MultiArray',  values: [0, 1, 2, 3, 32767, 65535, 4294967294, 4294967295]},
    {type: 'UInt64MultiArray',  values: [0, 1, 2, 3, 32767, 65535, Number.MAX_SAFE_INTEGER]},
  ].forEach((testData) => {
    const topic = testData.topic || 'topic' + testData.type;
    it('Test translation of ' + testData.type + ' msg, value ' + testData.values, function() {
      const node = rclnodejs.createNode('test_message_translation_node');
      const MessageType = 'std_msgs/msg/' + testData.type;
      const publisher = node.createPublisher(MessageType, topic);
      return new Promise((resolve, reject) => {
        const sub = node.createSubscription(MessageType, topic, (value) => {
          // For primitive types, msgs are defined as a single `.data` field
          if (deepEqual(value.data, testData.values)) {
            node.destroy();
            resolve();
          } else {
            node.destroy();
            reject('Expected: ' + testData.values + ', Got: ' + value.data);
          }
        });
        publisher.publish({
          layout: {
            dim: [
              {label: 'length',  size: 0, stride: 0},
            ],
            data_offset: 0,
          },
          data: testData.values,
        });
        rclnodejs.spin(node);
      });
    });
  });

});

// describe('Rclnodejs message translation: primitive types array - exception', function() {
//   this.timeout(60 * 1000);

//   before(function() {
//     return rclnodejs.init();
//   });

//   after(function() {
//     rclnodejs.shutdown();
//   });

//   [
//     {type: 'ByteMultiArray',    values: [0, 1, 2, 3, 255, 256]},
//     {type: 'ByteMultiArray',    values: [-1, 0, 1, 2, 3, 255]},
//     {type: 'ByteMultiArray',    values: [-100, 0, 1, 2, 3, 255]},
//     {type: 'ByteMultiArray',    values: [0, 1, 2, 3, 32767, Number.MAX_SAFE_INTEGER]},
//     {type: 'ByteMultiArray',    values: [0, 1, 2, 3, 32767, Number.POSITIVE_INFINITY]},

//     {type: 'Int8MultiArray',    values: [-128, -127, -2, -1, 0, 1, 2, 3, 127, 128]},
//     {type: 'Int8MultiArray',    values: [-128, -127, -2, -1, 0, 1, 2, 3, 127, 129]},
//     {type: 'Int8MultiArray',    values: [-128, -127, -2, -1, 0, 1, 2, 3, 127, 1000]},
//     {type: 'Int8MultiArray',    values: [-129, -127, -2, -1, 0, 1, 2, 3, 127]},
//     {type: 'Int8MultiArray',    values: [-1000, -127, -2, -1, 0, 1, 2, 3, 127]},
//     {type: 'Int8MultiArray',    values: [-Number.MAX_SAFE_INTEGER, -2, -1, 0, 1, 2, 3, 32767]},
//     {type: 'Int8MultiArray',    values: [Number.NEGATIVE_INFINITY, -2, -1, 0, 1, 2, 3, 32767]},

//     {type: 'Int16MultiArray',   values: [-32768, -2, -1, 0, 1, 2, 3, 32767, 32768]},
//     {type: 'Int16MultiArray',   values: [-32768, -2, -1, 0, 1, 2, 3, 32767, 32769]},
//     {type: 'Int16MultiArray',   values: [-32768, -2, -1, 0, 1, 2, 3, 32767, 100000]},
//     {type: 'Int16MultiArray',   values: [-32768, -2, -1, 0, 1, 2, 3, 32767, 1000000]},
//     {type: 'Int16MultiArray',   values: [-32768, -2, -1, 0, 1, 2, 3, 32767, Number.MAX_SAFE_INTEGER]},
//     {type: 'Int16MultiArray',   values: [-32768, -2, -1, 0, 1, 2, 3, 32767, Number.POSITIVE_INFINITY]},
//     {type: 'Int16MultiArray',   values: [-32769, -2, -1, 0, 1, 2, 3, 32767]},
//     {type: 'Int16MultiArray',   values: [-100000, -2, -1, 0, 1, 2, 3, 32767]},
//     {type: 'Int16MultiArray',   values: [-Number.MAX_SAFE_INTEGER, -2, -1, 0, 1, 2, 3, 32767]},
//     {type: 'Int16MultiArray',   values: [Number.NEGATIVE_INFINITY, -2, -1, 0, 1, 2, 3, 32767]},

//     {type: 'Int32MultiArray',   values: [-3, -2, -1, 0, 1, 2, 3, 32767, 2147483647, 2147483648]},
//     {type: 'Int32MultiArray',   values: [-2147483649, -2, -1, 0, 1, 2, 3, 32767, 2147483647]},
//     {type: 'Int32MultiArray',   values: [-3, -2, -1, 0, 1, 2, 3, 32767, 2147483647, Number.MAX_SAFE_INTEGER]},
//     {type: 'Int32MultiArray',   values: [-3, -2, -1, 0, 1, 2, 3, 32767, 2147483647, Number.POSITIVE_INFINITY]},
//     {type: 'Int32MultiArray',   values: [-Number.MAX_SAFE_INTEGER, -2, -1, 0, 1, 2, 3, 32767, 2147483647]},
//     {type: 'Int32MultiArray',   values: [Number.NEGATIVE_INFINITY, -2, -1, 0, 1, 2, 3, 32767, 2147483647]},

//     {type: 'UInt8MultiArray',   values: [0, 1, 2, 3, 127, 255, 256]},
//     {type: 'UInt8MultiArray',   values: [0, 1, 2, 3, 127, 255, 257]},
//     {type: 'UInt8MultiArray',   values: [0, 1, 2, 3, 127, 255, 1000]},
//     {type: 'UInt8MultiArray',   values: [0, 1, 2, 3, 127, 255, 10000]},
//     {type: 'UInt8MultiArray',   values: [0, 1, 2, 3, 127, 255, Number.MAX_SAFE_INTEGER]},
//     {type: 'UInt8MultiArray',   values: [0, 1, 2, 3, 127, 255, Number.POSITIVE_INFINITY]},
//     {type: 'UInt8MultiArray',   values: [-1, 1, 2, 3, 127, 255]},
//     {type: 'UInt8MultiArray',   values: [-100, 1, 2, 3, 127, 255]},
//     {type: 'UInt8MultiArray',   values: [-1000, 1, 2, 3, 127, 255]},
//     {type: 'UInt8MultiArray',   values: [-Number.MAX_SAFE_INTEGER, -2, -1, 0, 1, 2, 3]},
//     {type: 'UInt8MultiArray',   values: [Number.NEGATIVE_INFINITY, -2, -1, 0, 1, 2, 3]},

//     {type: 'UInt16MultiArray',  values: [0, 1, 2, 3, 32767, 65535, 65536]},
//     {type: 'UInt16MultiArray',  values: [0, 1, 2, 3, 32767, 65535, 100000]},
//     {type: 'UInt16MultiArray',  values: [0, 1, 2, 3, 32767, 65535, 1000000]},
//     {type: 'UInt16MultiArray',  values: [0, 1, 2, 3, 32767, 65535, Number.MAX_SAFE_INTEGER]},
//     {type: 'UInt16MultiArray',  values: [0, 1, 2, 3, 32767, 65535, Number.POSITIVE_INFINITY]},
//     {type: 'UInt16MultiArray',  values: [-1, 1, 2, 3, 32767, 65535]},
//     {type: 'UInt16MultiArray',  values: [-10000, 1, 2, 3, 32767, 65535]},
//     {type: 'UInt16MultiArray',  values: [-Number.MAX_SAFE_INTEGER, 1, 2, 3, 32767, 65535]},
//     {type: 'UInt16MultiArray',  values: [Number.NEGATIVE_INFINITY, 1, 2, 3, 32767, 65535]},

//     {type: 'UInt32MultiArray',  values: [0, 1, 2, 3, 32767, 65535, 4294967296]},
//     {type: 'UInt32MultiArray',  values: [0, 1, 2, 3, 32767, 65535, 4294967297]},
//     {type: 'UInt32MultiArray',  values: [0, 1, 2, 3, 32767, 65535, 10000000000]},
//     {type: 'UInt32MultiArray',  values: [0, 1, 2, 3, 32767, 65535, 100000000000]},
//     {type: 'UInt32MultiArray',  values: [0, 1, 2, 3, 32767, 65535, Number.MAX_SAFE_INTEGER]},
//     {type: 'UInt32MultiArray',  values: [0, 1, 2, 3, 32767, 65535, Number.POSITIVE_INFINITY]},
//     {type: 'UInt32MultiArray',  values: [-1, 1, 2, 3, 32767, 65535, 4294967295]},
//     {type: 'UInt32MultiArray',  values: [-2, 1, 2, 3, 32767, 65535, 4294967295]},
//     {type: 'UInt32MultiArray',  values: [-10000, 1, 2, 3, 32767, 65535, 4294967295]},
//     {type: 'UInt32MultiArray',  values: [-Number.MAX_SAFE_INTEGER, 1, 2, 3, 32767, 65535, 4294967295]},
//   ].forEach((testData) => {
//     const topic = testData.topic || 'topic' + testData.type;
//     const testCaseName = 'Test translation of ' + testData.type +
//                          ' msg, value ' + testData.values + ' (exception)';
//     it(testCaseName, function() {
//       const node = rclnodejs.createNode('test_message_translation_node');
//       const MessageType = 'std_msgs/msg/' + testData.type;
//       const publisher = node.createPublisher(MessageType, topic);
//       return new Promise((resolve, reject) => {
//         try {
//           publisher.publish({
//             layout: {
//               dim: [
//                 {label: 'length',  size: 0, stride: 0},
//               ],
//               data_offset: 0,
//             },
//             data: testData.values,
//           });
//           node.destroy();
//           reject('Exception is expected for: ' + testData.values);
//         } catch (e) {
//           node.destroy();
//           resolve();
//         }
//         rclnodejs.spin(node);
//       });
//     });
//   });
// });

describe('Rclnodejs message translation: TypedArray large data', function() {
  this.timeout(60 * 1000);

  before(function() {
    return rclnodejs.init();
  });

  after(function() {
    rclnodejs.shutdown();
  });

  const arrayLength = 100 * 1000;
  [
  /* eslint-disable max-len */
    {type: 'ByteMultiArray',    values: arrayGen.generateValues(Uint8Array,   arrayLength, 256, arrayGen.positive, Math.floor)},
    {type: 'Float32MultiArray', values: arrayGen.generateValues(Float32Array, arrayLength, 100000000, arrayGen.negative, arrayGen.noRound)},
    {type: 'Float32MultiArray', values: arrayGen.generateValues(Float32Array, arrayLength, 10000, arrayGen.negative, arrayGen.noRound)},
    {type: 'Float64MultiArray', values: arrayGen.generateValues(Float64Array, arrayLength, Number.MAX_VALUE, arrayGen.negative, arrayGen.noRound)},
    {type: 'Float64MultiArray', values: arrayGen.generateValues(Float64Array, arrayLength, 10000, arrayGen.negative, arrayGen.noRound)},
    {type: 'Int8MultiArray',    values: arrayGen.generateValues(Int8Array,    arrayLength, 128, arrayGen.negative, Math.floor)},
    {type: 'Int16MultiArray',   values: arrayGen.generateValues(Int16Array,   arrayLength, 32768, arrayGen.negative, Math.floor)},
    {type: 'Int32MultiArray',   values: arrayGen.generateValues(Int32Array,   arrayLength, 2147483648, arrayGen.negative, Math.floor)},
    {type: 'UInt8MultiArray',   values: arrayGen.generateValues(Uint8Array,   arrayLength, 256, arrayGen.positive, Math.floor)},
    {type: 'UInt16MultiArray',  values: arrayGen.generateValues(Uint16Array,  arrayLength, 65536, arrayGen.positive, Math.floor)},
    {type: 'UInt32MultiArray',  values: arrayGen.generateValues(Uint32Array,  arrayLength, 4294967296, arrayGen.positive, Math.floor)},

    {type: 'ByteMultiArray',    values: arrayGen.generateValues(Array, arrayLength, 256, arrayGen.positive, Math.floor)},
    // Note: According to IEEE 754, float32 has 6 significant decimal digits, skip float32 for now
    // {type: 'Float32MultiArray', values: arrayGen.generateValues(Array, arrayLength, 100000000, arrayGen.negative, arrayGen.noRound)},
    // {type: 'Float32MultiArray', values: arrayGen.generateValues(Array, arrayLength, 10000, arrayGen.negative, arrayGen.noRound)},
    {type: 'Float64MultiArray', values: arrayGen.generateValues(Array, arrayLength, Number.MAX_VALUE, arrayGen.negative, arrayGen.noRound)},
    {type: 'Float64MultiArray', values: arrayGen.generateValues(Array, arrayLength, 10000, arrayGen.negative, arrayGen.noRound)},
    {type: 'Int8MultiArray',    values: arrayGen.generateValues(Array, arrayLength, 128, arrayGen.negative, Math.floor)},
    {type: 'Int16MultiArray',   values: arrayGen.generateValues(Array, arrayLength, 32768, arrayGen.negative, Math.floor)},
    {type: 'Int32MultiArray',   values: arrayGen.generateValues(Array, arrayLength, 2147483648, arrayGen.negative, Math.floor)},
    {type: 'UInt8MultiArray',   values: arrayGen.generateValues(Array, arrayLength, 256, arrayGen.positive, Math.floor)},
    {type: 'UInt16MultiArray',  values: arrayGen.generateValues(Array, arrayLength, 65536, arrayGen.positive, Math.floor)},
    {type: 'UInt32MultiArray',  values: arrayGen.generateValues(Array, arrayLength, 4294967296, arrayGen.positive, Math.floor)},
  /* eslint-enable max-len */
  ].forEach((testData) => {
    const topic = testData.topic || 'topic' + testData.type;
    it('Test translation of ' + testData.type + ' msg, number of values ' + testData.values.length, function() {
      const node = rclnodejs.createNode('test_message_translation_node');
      const MessageType = 'std_msgs/msg/' + testData.type;
      const publisher = node.createPublisher(MessageType, topic);
      return new Promise((resolve, reject) => {
        const sub = node.createSubscription(MessageType, topic, (value) => {
          // For primitive types, msgs are defined as a single `.data` field
          if (deepEqual(value.data, testData.values)) {
            node.destroy();
            resolve();
          } else {
            node.destroy();
            reject('Expected: ' + testData.values + ',                              Got: ' + value.data);
          }
        });
        publisher.publish({
          layout: {
            dim: [
              {label: 'length',  size: 0, stride: 0},
            ],
            data_offset: 0,
          },
          data: testData.values,
        });
        rclnodejs.spin(node);
      });
    });
  });
});
