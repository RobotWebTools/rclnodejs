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

// Usage: lanuch ./install/bin/examples_rclcpp_minimal_subscriber_lambda to display the message published by this program

'use strict';

const rclnodejs = require('../index.js');
const Message = rclnodejs.Message;

rclnodejs.init();

let node = rclnodejs.createNode('publisher_example_node');

const messageType = {
  pkgName: 'std_msgs',
  msgSubfolder: 'msg',
  msgName: 'String',
};

let publisher = node.createPublisher(messageType, 'topic');

let counter = 0;
setInterval(function() {
  const testMessage = 'hello rclnodejs ' + counter++;

  Message.createMessage(messageType).then(([message, spec]) => {
    // console.log(message, spec);
    message.data = testMessage;

    // const ffi = require('ffi');
    const StructType = require('ref-struct');
    const ref = require('ref');
    let size_t = ref.types.size_t;
    // const charPtr = ref.refType('char');

    const rosidl_generator_c__String = StructType({
      data: ref.types.CString,
      size: size_t,
      capacity: size_t,
    });

    const std_msgs__msg__String = StructType({
      data: rosidl_generator_c__String,
    });

    function rosidl_generator_c__String__assign(str, text) {
      let strBuf = new Buffer(text);
      strBuf.type = ref.types.CString;
      str.data = strBuf;
      str.size = text.length;
      str.capacity = text.length + 1;
    }

    function getBuffer(obj) {
      return obj['ref.buffer'];
    }

    let msg = new std_msgs__msg__String();
    rosidl_generator_c__String__assign(msg.data, testMessage);
    // console.log(msg);

    // const libstd_msgs__rosidl_generator_c = ffi.Library('libstd_msgs__rosidl_generator_c', {
    //   'rosidl_generator_c__String__Array__create': [ref.refType(rosidl_generator_c__String), [size_t]],
    //   'rosidl_generator_c__String__assign': [ref.types.bool, [ref.refType(rosidl_generator_c__String), ref.types.CString]]
    // });

    // let ptr = libstd_msgs__rosidl_generator_c.rosidl_generator_c__String__Array__create(1);
    // let buf = new Buffer(testMessage);
    // buf.type = ref.types.CString;
    // libstd_msgs__rosidl_generator_c.rosidl_generator_c__String__assign(ptr, buf);
    // // console.log('JS Object:', ref.deref(ptr));
    // // console.log('Original pointer:', ptr);

    console.log('Publishing message:', testMessage);

    publisher.publish(getBuffer(msg));
    // publisher.publish(msg['ref.buffer']);
    // publisher.publish(ptr);
  });

  // console.log('Publishing message:', testMessage);
  // publisher.publishStringMessage(testMessage);
}, 100);

rclnodejs.spin(node);

// rclnodejs.shutdown();