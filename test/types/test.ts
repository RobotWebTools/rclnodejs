/// <reference path='../../types/index.d.ts' />
import * as rclnodejs from 'rclnodejs';

// TODO - reinstate the disabled (commented out) test due to
// somewhat false negatives reported by multiple version of typescript
// and undefined rules for how union type ordering

// // ---- Uint8Array ----
// const u8arr = rclnodejs.require(
//   'example_interfaces.msg.UInt8MultiArray'
// ) as rclnodejs.example_interfaces.msg.UInt8MultiArray;
// // $ExpectType Uint8Array | number[]
// u8arr.data;

// // ---- Uint8Array ----
// const u8arrx = rclnodejs.require(
//   'std_msgs.msg.UInt8MultiArray'
// ) as rclnodejs.std_msgs.msg.UInt8MultiArray;
// // $ExpectType Uint8Array | number[]
// u8arrx.data;
