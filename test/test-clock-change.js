'use strict';

const assert = require('assert');
const rclnodejs = require('../index.js');
const { ClockChange } = rclnodejs;

describe('ClockChange enum', function () {
  it('should define all clock change constants', function () {
    assert.strictEqual(typeof ClockChange, 'object');
    assert.strictEqual(typeof ClockChange.ROS_TIME_NO_CHANGE, 'number');
    assert.strictEqual(typeof ClockChange.ROS_TIME_ACTIVATED, 'number');
    assert.strictEqual(typeof ClockChange.ROS_TIME_DEACTIVATED, 'number');
    assert.strictEqual(typeof ClockChange.SYSTEM_TIME_NO_CHANGE, 'number');
  });

  it('should have correct numeric values', function () {
    assert.strictEqual(ClockChange.ROS_TIME_NO_CHANGE, 1);
    assert.strictEqual(ClockChange.ROS_TIME_ACTIVATED, 2);
    assert.strictEqual(ClockChange.ROS_TIME_DEACTIVATED, 3);
    assert.strictEqual(ClockChange.SYSTEM_TIME_NO_CHANGE, 4);
  });

  it('should match RCL constants', function () {
    // These values should match the C/C++ rcl_clock_change_t enum
    // RCL_ROS_TIME_NO_CHANGE = 1
    // RCL_ROS_TIME_ACTIVATED = 2
    // RCL_ROS_TIME_DEACTIVATED = 3
    // RCL_SYSTEM_TIME_NO_CHANGE = 4
    assert.strictEqual(ClockChange.ROS_TIME_NO_CHANGE, 1);
    assert.strictEqual(ClockChange.ROS_TIME_ACTIVATED, 2);
    assert.strictEqual(ClockChange.ROS_TIME_DEACTIVATED, 3);
    assert.strictEqual(ClockChange.SYSTEM_TIME_NO_CHANGE, 4);
  });

  it('should be exportable from rclnodejs module', function () {
    const { ClockChange: ImportedClockChange } = require('../index.js');
    assert.strictEqual(ImportedClockChange, ClockChange);
    assert.strictEqual(ImportedClockChange.ROS_TIME_ACTIVATED, 2);
  });

  it('should have all expected properties', function () {
    const expectedKeys = [
      'ROS_TIME_NO_CHANGE',
      'ROS_TIME_ACTIVATED',
      'ROS_TIME_DEACTIVATED',
      'SYSTEM_TIME_NO_CHANGE',
    ];

    const actualKeys = Object.keys(ClockChange);
    expectedKeys.forEach((key) => {
      assert.ok(
        actualKeys.includes(key),
        `ClockChange should have property ${key}`
      );
    });
  });

  it('should have distinct values', function () {
    const values = Object.values(ClockChange);
    const uniqueValues = new Set(values);
    assert.strictEqual(
      values.length,
      uniqueValues.size,
      'All ClockChange values should be unique'
    );
  });
});
