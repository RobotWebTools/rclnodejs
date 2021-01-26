'use strict';

/**
 * Benchmarks the performance in converting from to javascript objects to native C structs.
 * Requires the ros package "test_msgs" to be available.
 */
const app = require('commander');
const generatorOptions = require('../../../generated/generator-options');

app.option('-r, --runs <n>', 'Number of times to run').parse(process.argv);
const runs = app.runs || 1;

const BasicTypes = require('../../../generated/test_msgs/test_msgs__msg__BasicTypes');

if (generatorOptions.idlProvider === 'rosidl') {
  const jsObj = {
    bool_value: false,
    byte_value: 0,
    char_value: 0,
    float32_value: 0,
    float64_value: 0,
    int8_value: 0,
    uint8_value: 0,
    int16_value: 0,
    uint16_value: 0,
    int32_value: 0,
    uint32_value: 0,
    int64_value: BigInt(0),
    uint64_value: BigInt(0),
  };
  const startTime = process.hrtime();
  for (let i = 0; i < runs; i++) {
    BasicTypes.toRosMessage(jsObj);
  }
  const timeTaken = process.hrtime(startTime);
  console.log(
    `Benchmark took ${timeTaken[0]} seconds and ${Math.ceil(
      timeTaken[1] / 1000000
    )} milliseconds.`
  );
} else {
  const jsObj = {
    bool_value: false,
    byte_value: 0,
    char_value: 0,
    float32_value: 0,
    float64_value: 0,
    int8_value: 0,
    uint8_value: 0,
    int16_value: 0,
    uint16_value: 0,
    int32_value: 0,
    uint32_value: 0,
    int64_value: 0,
    uint64_value: 0,
  };
  const startTime = process.hrtime();
  for (let i = 0; i < runs; i++) {
    const msg = new BasicTypes(jsObj);
    msg.serialize();
  }
  const timeTaken = process.hrtime(startTime);
  console.log(
    `Benchmark took ${timeTaken[0]} seconds and ${Math.ceil(
      timeTaken[1] / 1000000
    )} milliseconds.`
  );
}
