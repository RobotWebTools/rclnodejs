/// <reference path='../../types/index.d.ts' />
import * as rclnodejs from 'rclnodejs';


// ---- Uint8Array ----
const u8arr = rclnodejs.require('example_interfaces.msg.UInt8MultiArray') as rclnodejs.example_interfaces.msg.UInt8MultiArray;
// $ExpectType number[] | Uint8Array
u8arr.data;

// ---- Uint8Array ----
const u8arrx = rclnodejs.require('std_msgs.msg.UInt8MultiArray') as rclnodejs.std_msgs.msg.UInt8MultiArray;
// $ExpectType number[] | Uint8Array
u8arrx.data;
