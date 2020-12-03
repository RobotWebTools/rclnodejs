'use strict';

const rclnodejs = require('.');
const test = require('bindings')('std_msgs');
console.log(test.String.create());

// rclnodejs.init().then(() => {
//   const node = rclnodejs.createNode('test');
//   node.createSubscription('std_msgs/msg/String', 'test', (msg) => {
//     console.log(msg);
//   });
//   rclnodejs.spin(node);
// });
