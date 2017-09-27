#!/usr/bin/env python3
# Copyright (c) 2017 Intel Corporation. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http:#www.apache.org/licenses/LICENSE-2.0
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
import signal

node = None

def cleanup():
  global node
  node.destroy_node()
  rclpy.shutdown()

def handler(signum, frame):
  cleanup()
  sys.exit(0)

def callback(msg):
  sys.stdout.write(str(msg.data))
  sys.stdout.flush()

def main():
  global node
  rclType = sys.argv[1]
  signal.signal(signal.SIGINT, handler)
  
  rclpy.init()

  if rclType == 'Bool':
    node = rclpy.create_node('py_bool_subscription')
    subscription = node.create_subscription(Bool, 'Bool_js_py_channel', callback)
  elif rclType == 'Byte':
    node = rclpy.create_node('py_byte_subscription')
    subscription = node.create_subscription(Byte, 'Byte_js_py_channel', callback)
  elif rclType == 'Char':
    node = rclpy.create_node('py_char_subscription')
    subscription = node.create_subscription(Char, 'Char_js_py_channel', callback)
  elif rclType == 'String':
    node = rclpy.create_node('py_string_subscription')
    subscription = node.create_subscription(String, 'String_js_py_channel', callback)
  elif rclType == 'Int8':
    node = rclpy.create_node('py_int8_subscription')
    subscription = node.create_subscription(Int8, 'Int8_js_py_channel', callback)
  elif rclType == 'UInt8':
    node = rclpy.create_node('py_uint8_subscription')
    subscription = node.create_subscription(UInt8, 'UInt8_js_py_channel', callback)
  elif rclType == 'Int16':
    node = rclpy.create_node('py_int16_subscription')
    subscription = node.create_subscription(Int16, 'Int16_js_py_channel', callback)
  elif rclType == 'UInt16':
    node = rclpy.create_node('py_uint16_subscription')
    subscription = node.create_subscription(UInt16, 'UInt16_js_py_channel', callback)
  elif rclType == 'Int32':
    node = rclpy.create_node('py_int32_subscription')
    subscription = node.create_subscription(Int32, 'Int32_js_py_channel', callback)
  elif rclType == 'UInt32':
    node = rclpy.create_node('py_uint32_subscription')
    subscription = node.create_subscription(UInt32, 'UInt32_js_py_channel', callback)
  elif rclType == 'Int64':
    node = rclpy.create_node('py_int64_subscription')
    subscription = node.create_subscription(Int64, 'Int64_js_py_channel', callback)
  elif rclType == 'UInt64':
    node = rclpy.create_node('py_uint64_subscription')
    subscription = node.create_subscription(UInt64, 'UInt64_js_py_channel', callback)
  elif rclType == 'Float32':
    node = rclpy.create_node('py_float32_subscription')
    subscription = node.create_subscription(Float32, 'Float32_js_py_channel', callback)
  elif rclType == 'Float64':
    node = rclpy.create_node('py_float64_subscription')
    subscription = node.create_subscription(Float64, 'Float64_js_py_channel', callback)                               
  while rclpy.ok():
    rclpy.spin_once(node)

  cleanup()


if __name__ == '__main__':
  main()
