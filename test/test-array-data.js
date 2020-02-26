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
const childProcess = require('child_process');
const deepEqual = require('deep-equal');
const rclnodejs = require('../index.js');
const translator = require('../rosidl_gen/message_translator.js');
const arrayGen = require('./array_generator.js');

function isTypedArray(v) {
  return ArrayBuffer.isView(v) && !(v instanceof DataView);
}

describe('rclnodejs message communication', function() {
  this.timeout(60 * 1000);

  before(function() {
    return rclnodejs.init();
  });

  after(function() {
    rclnodejs.shutdown();
  });

  /* eslint-disable camelcase */
  /* eslint-disable key-spacing */
  /* eslint-disable comma-spacing */

  const layout = {
    dim: [
      { label: 'height', size: 10, stride: 600 },
      { label: 'width', size: 20, stride: 60 },
    ],
    data_offset: 0,
  };

  [
    {
      pkg: 'std_msgs',
      type: 'Int8MultiArray',
      values: [
        { layout: layout, data: [-10, 1, 2, 3, 8, 6, 0, -25] }, // Provide data via Array
        { layout: layout, data: Int8Array.from([-10, 1, 2, 3, 8, 6, 0, -25]) }, // Provide data via TypedArray
      ],
    },
    {
      pkg: 'std_msgs',
      type: 'Int16MultiArray',
      values: [
        { layout: layout, data: [-10, 1, 2, 3, 8, 6, 0, -25] }, // Provide data via Array
        { layout: layout, data: Int16Array.from([-10, 1, 2, 3, 8, 6, 0, -25]) }, // Provide data via TypedArray
      ],
    },
    {
      pkg: 'std_msgs',
      type: 'Int32MultiArray',
      values: [
        { layout: layout, data: [-10, 1, 2, 3, 8, 6, 0, -25] }, // Provide data via Array
        { layout: layout, data: Int32Array.from([-10, 1, 2, 3, 8, 6, 0, -25]) }, // Provide data via TypedArray
      ],
    },
    {
      pkg: 'std_msgs',
      type: 'Int64MultiArray',
      values: [
        {
          layout: layout,
          data: [-111, 1, 2, 3, 8, 6, 0, -25, Number.MAX_SAFE_INTEGER],
        }, // Provide data via Array
      ],
    },
    {
      pkg: 'std_msgs',
      type: 'ByteMultiArray',
      values: [
        { layout: layout, data: [0, 1, 2, 3, 8, 6, 0, 255] }, // Provide data via Array
        { layout: layout, data: Uint8Array.from([0, 1, 2, 3, 8, 6, 0, 255]) }, // Provide data via TypedArray
      ],
    },
    {
      pkg: 'std_msgs',
      type: 'UInt8MultiArray',
      values: [
        { layout: layout, data: [0, 1, 2, 3, 8, 6, 0, 255] }, // Provide data via Array
        { layout: layout, data: Uint8Array.from([0, 1, 2, 3, 8, 6, 0, 255]) }, // Provide data via TypedArray
      ],
    },
    {
      pkg: 'std_msgs',
      type: 'UInt16MultiArray',
      values: [
        { layout: layout, data: [0, 1, 2, 3, 8, 6, 0, 255] }, // Provide data via Array
        { layout: layout, data: Uint16Array.from([0, 1, 2, 3, 8, 6, 0, 255]) }, // Provide data via TypedArray
      ],
    },
    {
      pkg: 'std_msgs',
      type: 'UInt32MultiArray',
      values: [
        { layout: layout, data: [0, 1, 2, 3, 8, 6, 0, 255] }, // Provide data via Array
        { layout: layout, data: Uint32Array.from([0, 1, 2, 3, 8, 6, 0, 255]) }, // Provide data via TypedArray
      ],
    },
    {
      pkg: 'std_msgs',
      type: 'UInt64MultiArray',
      values: [
        {
          layout: layout,
          data: [0, 1, 2, 3, 8, 6, 0, 255, Number.MAX_SAFE_INTEGER],
        }, // Provide data via Array
      ],
    },
    {
      pkg: 'std_msgs',
      type: 'Float32MultiArray',
      values: [
        { layout: layout, data: [-10, 1, 2, 3, 8, 6, 0, -25] }, // Provide data via Array
        {
          layout: layout,
          data: Float32Array.from([-10, 1, 2, 3, 8, 6, 0, -25]),
        }, // Provide data via TypedArray
      ],
    },
    {
      pkg: 'std_msgs',
      type: 'Float64MultiArray',
      values: [
        { layout: layout, data: [-10, 1, 2, 3, 8, 6, 0, -25] }, // Provide data via Array
        {
          layout: layout,
          data: Float64Array.from([-10, 1, 2, 3, 8, 6, 0, -25]),
        }, // Provide data via TypedArray
      ],
    },
    /* eslint-enable camelcase */
    /* eslint-enable key-spacing */
    /* eslint-enable comma-spacing */
  ].forEach(testData => {
    const topic = testData.topic || 'topic' + testData.type;
    testData.values.forEach((v, i) => {
      it('Test ' + testData.type + '.copy()' + ', case ' + i, function() {
        const MessageType = rclnodejs.require(
          testData.pkg + '/msg/' + testData.type
        );
        const msg1 = translator.toROSMessage(MessageType, v);
        const msg2 = new MessageType();
        msg2.copy(msg1);

        function checkMessage(msg) {
          assert(typeof msg.layout === 'object');

          assert(Array.isArray(msg.layout.dim.data));
          // First element
          assert(typeof msg.layout.dim.data[0] === 'object');
          assert(msg.layout.dim.data[0].label === v.layout.dim[0].label);
          assert(msg.layout.dim.data[0].size === v.layout.dim[0].size);
          assert(msg.layout.dim.data[0].stride === v.layout.dim[0].stride);

          // Second element
          assert(typeof msg.layout.dim.data[1] === 'object');
          assert(msg.layout.dim.data[1].label === v.layout.dim[1].label);
          assert(msg.layout.dim.data[1].size === v.layout.dim[1].size);
          assert(msg.layout.dim.data[1].stride === v.layout.dim[1].stride);

          assert(msg.layout.data_offset === v.layout.data_offset);

          assert(msg.data);
          if (isTypedArray(v.data)) {
            assert.deepStrictEqual(msg.data, v.data);
          } else {
            assert.ok(deepEqual(msg.data, v.data));
          }
        }

        checkMessage(msg1);
        checkMessage(msg2);

        const o = v.data[0];
        assert(msg1.data[0] == o);
        assert(msg2.data[0] == o);

        const r = Math.round(Math.random() * 100);
        msg1.data[0] = r;

        assert(msg1.data[0] == r);
        assert(msg2.data[0] == o);
      });
    });
  });

  const uint8Data = arrayGen.generateValues(
    Array,
    320 * 240 * 4 * 4,
    256,
    arrayGen.positive,
    Math.floor
  );
  const float32Data = arrayGen.generateValues(
    Array,
    5000,
    1.0,
    arrayGen.positive,
    arrayGen.noRound
  );
  const float64Data = arrayGen.generateValues(
    Array,
    5000,
    1.0,
    arrayGen.positive,
    arrayGen.noRound
  );

  [
    /* eslint-disable camelcase */
    /* eslint-disable key-spacing */
    /* eslint-disable comma-spacing */
    {
      pkg: 'sensor_msgs',
      type: 'PointCloud2',
      arrayType: Uint8Array,
      property: 'data',
      values: [
        {
          header: { stamp: { sec: 11223, nanosec: 44556 }, frame_id: 'f001' },
          height: 240,
          width: 320,
          fields: [],
          is_bigendian: false,
          point_step: 16,
          row_step: 320 * 16,
          data: uint8Data,
          is_dense: false,
        },
        {
          header: { stamp: { sec: 11223, nanosec: 44556 }, frame_id: 'f001' },
          height: 240,
          width: 320,
          fields: [],
          is_bigendian: false,
          point_step: 16,
          row_step: 320 * 16,
          data: Uint8Array.from(uint8Data),
          is_dense: false,
        },
      ],
    },
    {
      pkg: 'sensor_msgs',
      type: 'Image',
      arrayType: Uint8Array,
      property: 'data',
      values: [
        {
          header: { stamp: { sec: 11223, nanosec: 44556 }, frame_id: 'f001' },
          height: 240,
          width: 320,
          encoding: 'rgba',
          is_bigendian: false,
          step: 320 * 16,
          is_dense: false,
          data: uint8Data,
        },
        {
          header: { stamp: { sec: 11223, nanosec: 44556 }, frame_id: 'f001' },
          height: 240,
          width: 320,
          encoding: 'rgba',
          is_bigendian: false,
          step: 320 * 16,
          is_dense: false,
          data: Uint8Array.from(uint8Data),
        },
      ],
    },
    {
      pkg: 'sensor_msgs',
      type: 'CompressedImage',
      arrayType: Uint8Array,
      property: 'data',
      values: [
        {
          header: { stamp: { sec: 11223, nanosec: 44556 }, frame_id: 'f001' },
          format: 'jpeg',
          data: uint8Data,
        },
        {
          header: { stamp: { sec: 11223, nanosec: 44556 }, frame_id: 'f001' },
          format: 'jpeg',
          data: Uint8Array.from(uint8Data),
        },
      ],
    },
    {
      pkg: 'sensor_msgs',
      type: 'ChannelFloat32',
      arrayType: Float32Array,
      property: 'values',
      values: [
        { name: 'intensity', values: float32Data },
        { name: 'intensity', values: Float32Array.from(float32Data) },
        {
          name: 'intensity',
          values: Float32Array.from(Float32Array.from(float32Data)),
        },
      ],
    },
    {
      pkg: 'sensor_msgs',
      type: 'JointState',
      arrayType: Float64Array,
      property: 'position',
      values: [
        {
          header: {
            stamp: { sec: 123456, nanosec: 789 },
            frame_id: 'main frame',
          },
          name: ['Tom', 'Jerry'],
          velocity: [2, 3],
          effort: [4, 5, 6],
          position: float64Data,
        },
        {
          header: {
            stamp: { sec: 123456, nanosec: 789 },
            frame_id: 'main frame',
          },
          name: ['Tom', 'Jerry'],
          velocity: [2, 3],
          effort: [4, 5, 6],
          position: Float64Array.from(float64Data),
        },
        {
          header: {
            stamp: { sec: 123456, nanosec: 789 },
            frame_id: 'main frame',
          },
          name: ['Tom', 'Jerry'],
          velocity: [2, 3],
          effort: [4, 5, 6],
          position: Float64Array.from(Float64Array.from(float64Data)),
        },
      ],
    },
    /* eslint-enable camelcase */
    /* eslint-enable key-spacing */
    /* eslint-enable comma-spacing */
  ].forEach(testData => {
    const topic = testData.topic || 'topic' + testData.type;
    testData.values.forEach((v, i) => {
      it(
        'Make sure ' + testData.type + ' use TypedArray' + ', case ' + i,
        function() {
          const MessageType = rclnodejs.require(
            testData.pkg + '/msg/' + testData.type
          );
          const msg = translator.toROSMessage(MessageType, v);
          assert(isTypedArray(msg[testData.property]));
          assert(msg[testData.property] instanceof testData.arrayType);

          msg[testData.property] = v[testData.property];
          assert(isTypedArray(msg[testData.property]));
          assert(msg[testData.property] instanceof testData.arrayType);

          const msg2 = new MessageType();
          assert(isTypedArray(msg2[testData.property]));
          assert(msg2[testData.property] instanceof testData.arrayType);
        }
      );
    });
  });

  const arrayLength = 1024;
  [
    /* eslint-disable max-len */
    {
      type: 'ByteMultiArray',
      arrayType: Uint8Array,
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
      arrayType: Float32Array,
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
      arrayType: Float64Array,
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
      arrayType: Int8Array,
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
      arrayType: Int16Array,
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
      arrayType: Int32Array,
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
      arrayType: Uint8Array,
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
      arrayType: Uint16Array,
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
      arrayType: Uint32Array,
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
      arrayType: Uint8Array,
      values: arrayGen.generateValues(
        Array,
        arrayLength,
        256,
        arrayGen.positive,
        Math.floor
      ),
    },
    {
      type: 'Float32MultiArray',
      arrayType: Float32Array,
      values: arrayGen.generateValues(
        Array,
        arrayLength,
        10000,
        arrayGen.negative,
        arrayGen.noRound
      ),
    },
    {
      type: 'Float64MultiArray',
      arrayType: Float64Array,
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
      arrayType: Int8Array,
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
      arrayType: Int16Array,
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
      arrayType: Int32Array,
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
      arrayType: Uint8Array,
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
      arrayType: Uint16Array,
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
      arrayType: Uint32Array,
      values: arrayGen.generateValues(
        Array,
        arrayLength,
        4294967296,
        arrayGen.positive,
        Math.floor
      ),
    },
    /* eslint-enable max-len */
  ].forEach(testData => {
    const topic = testData.topic || 'topic' + testData.type;
    it('Make sure ' + testData.type + ' use TypedArray', function() {
      const MessageType = rclnodejs.require('std_msgs/msg/' + testData.type);
      const msg = translator.toROSMessage(MessageType, {
        layout: {
          dim: [{ label: 'length', size: 0, stride: 0 }],
          // eslint-disable-next-line
          data_offset: 0,
        },
        data: testData.values,
      });
      assert(isTypedArray(msg.data));
      assert(msg.data instanceof testData.arrayType);

      msg.data = testData.values;
      assert(isTypedArray(msg.data));
      assert(msg.data instanceof testData.arrayType);

      const msg2 = new MessageType();
      assert(isTypedArray(msg2.data));
      assert(msg2.data instanceof testData.arrayType);
    });
  });
});
