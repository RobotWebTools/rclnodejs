#!/usr/bin/env python3
# Copyright (c) 2017 Intel Corporation. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import rclpy
from time import sleep
from std_msgs.msg import *
from builtin_interfaces.msg import *
from sensor_msgs.msg import *
import signal

node = None
publisher = None

def cleanup():
  global node
  node.destroy_node()
  rclpy.shutdown()

def handler(signum, frame):
  cleanup()
  sys.exit(0)

def callback(msg):
  # sys.stdout.write(str(msg.data))
  # sys.stdout.flush()
  global publisher
  publisher.publish(msg)

def callback_array(msg):
  sys.stdout.write(''.join([r.decode('utf-8') for r in msg.data]))
  sys.stdout.flush()

def callback_colorrgba(msg):
  sys.stdout.write('(' + str(msg.r) + ',' 
                    + str(msg.g) + ',' 
                    + str(msg.b) + ','
                    + str(msg.a) + ')')
  sys.stdout.flush()

def callback_header(msg):
  sys.stdout.write('(' + str(msg.stamp.sec) + ','
                    + str(msg.stamp.nanosec) + ','
                    + str(msg.frame_id) + ')')
  sys.stdout.flush()

def callback_jointstate(msg):
  sys.stdout.write('(' + str(msg.header.stamp.sec) + ','
                    + str(msg.header.stamp.nanosec) + ','
                    + str(msg.header.frame_id) + ','
                    + str(msg.name) + ','
                    + str(msg.position) + ','
                    + str(msg.velocity) + ','
                    + str(msg.effort)
                    + ')')
  sys.stdout.flush()

def main():
  global node
  global publisher
  rclType = sys.argv[1]
  signal.signal(signal.SIGINT, handler)
  
  rclpy.init()

  if rclType == 'Bool':
    node = rclpy.create_node('py_bool_subscription')
    publisher = node.create_publisher(Bool, 'Bool_js_py_back_channel', 10)
    subscription = node.create_subscription(Bool, 'Bool_js_py_channel', callback, 10)
  elif rclType == 'Byte':
    node = rclpy.create_node('py_byte_subscription')
    publisher = node.create_publisher(Byte, 'Byte_js_py_back_channel', 10);
    subscription = node.create_subscription(Byte, 'Byte_js_py_channel', callback, 10)
  elif rclType == 'Char':
    node = rclpy.create_node('py_char_subscription')
    publisher = node.create_publisher(Char, 'Char_js_py_back_channel', 10)
    subscription = node.create_subscription(Char, 'Char_js_py_channel', callback, 10)
  elif rclType == 'String':
    node = rclpy.create_node('py_string_subscription')
    publisher = node.create_publisher(String, 'String_js_py_back_channel', 10)
    subscription = node.create_subscription(String, 'String_js_py_channel', callback, 10)
  elif rclType == 'Int8':
    node = rclpy.create_node('py_int8_subscription')
    publisher = node.create_publisher(Int8, 'Int8_js_py_back_channel', 10)
    subscription = node.create_subscription(Int8, 'Int8_js_py_channel', callback, 10)
  elif rclType == 'UInt8':
    node = rclpy.create_node('py_uint8_subscription')
    publisher = node.create_publisher(UInt8, 'UInt8_js_py_back_channel', 10)
    subscription = node.create_subscription(UInt8, 'UInt8_js_py_channel', callback, 10)
  elif rclType == 'Int16':
    node = rclpy.create_node('py_int16_subscription')
    publisher = node.create_publisher(Int16, 'Int16_js_py_back_channel', 10)
    subscription = node.create_subscription(Int16, 'Int16_js_py_channel', callback, 10)
  elif rclType == 'UInt16':
    node = rclpy.create_node('py_uint16_subscription')
    publisher = node.create_publisher(UInt16, 'UInt16_js_py_back_channel', 10)
    subscription = node.create_subscription(UInt16, 'UInt16_js_py_channel', callback, 10)
  elif rclType == 'Int32':
    node = rclpy.create_node('py_int32_subscription')
    publisher = node.create_publisher(Int32, 'Int32_js_py_back_channel', 10)
    subscription = node.create_subscription(Int32, 'Int32_js_py_channel', callback, 10)
  elif rclType == 'UInt32':
    node = rclpy.create_node('py_uint32_subscription')
    publisher = node.create_publisher(UInt32, 'UInt32_js_py_back_channel', 10)
    subscription = node.create_subscription(UInt32, 'UInt32_js_py_channel', callback, 10)
  elif rclType == 'Int64':
    node = rclpy.create_node('py_int64_subscription')
    publisher = node.create_publisher(Int64, 'Int64_js_py_back_channel', 10)
    subscription = node.create_subscription(Int64, 'Int64_js_py_channel', callback, 10)
  elif rclType == 'UInt64':
    node = rclpy.create_node('py_uint64_subscription')
    publisher = node.create_publisher(UInt64, 'UInt64_js_py_back_channel', 10)
    subscription = node.create_subscription(UInt64, 'UInt64_js_py_channel', callback, 10)
  elif rclType == 'Float32':
    node = rclpy.create_node('py_float32_subscription')
    publisher = node.create_publisher(Float32, 'Float32_js_py_back_channel', 10)
    subscription = node.create_subscription(Float32, 'Float32_js_py_channel', callback, 10)
  elif rclType == 'Float64':
    node = rclpy.create_node('py_float64_subscription')
    publisher = node.create_publisher(Float64, 'Float64_js_py_back_channel', 10)
    subscription = node.create_subscription(Float64, 'Float64_js_py_channel', callback, 10)
  elif rclType == 'Array':
    node = rclpy.create_node('py_array_subscription')
    publisher = node.create_publisher(ByteMultiArray, 'Array_js_py_back_channel', 10)
    subscription = node.create_subscription(ByteMultiArray, 'Array_js_py_channel', callback, 10)
  elif rclType == 'ColorRGBA':
    node = rclpy.create_node('py_colorrgba_subscription')
    publisher = node.create_publisher(ColorRGBA, 'ColorRGBA_js_py_back_channel', 10)
    subscription = node.create_subscription(ColorRGBA, 'ColorRGBA_js_py_channel', callback, 10)
  elif rclType == 'Header':
    node = rclpy.create_node('py_header_subscription')
    publisher = node.create_publisher(Header, 'Header_js_py_back_channel', 10)
    subscription = node.create_subscription(Header, 'Header_js_py_channel', callback, 10)
  elif rclType == 'JointState':
    node = rclpy.create_node('py_jointstate_subscrption')
    publisher = node.create_publisher(JointState, 'JointState_js_py_back_channel', 10)
    subscription = node.create_subscription(JointState, 'JointState_js_py_channel', callback, 10)
  while rclpy.ok():
    rclpy.spin_once(node)

  cleanup()


if __name__ == '__main__':
  main()
