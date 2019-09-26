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
import signal
from time import sleep

import rclpy
from std_msgs.msg import *
from builtin_interfaces.msg import *
from sensor_msgs.msg import *

node = None

def cleanup():
  global node
  node.destroy_node()
  rclpy.shutdown()

def handler(signum, frame):
  cleanup()
  sys.exit(0)

def main():
  global node
  rclType = sys.argv[1]
  signal.signal(signal.SIGINT, handler)

  rclpy.init()
  
  if rclType == 'Bool':
    node = rclpy.create_node('py_bool_publisher')
    publisher = node.create_publisher(Bool, 'Bool_py_js_channel', 10)
    msg = Bool()
    msg.data = True
  elif rclType == 'Byte':
    node = rclpy.create_node('py_byte_publisher')
    publisher = node.create_publisher(Byte, 'Byte_py_js_channel', 10)
    msg = Byte()
    msg.data = b'\xff'
  elif rclType == 'Char':
    node = rclpy.create_node('py_char_publisher')
    publisher = node.create_publisher(Char, 'Char_py_js_channel', 10)
    msg = Char()
    msg.data = 'A'
  elif rclType == 'String':
    node = rclpy.create_node('py_string_publisher')
    publisher = node.create_publisher(String, 'String_py_js_channel', 10)
    msg = String()
    msg.data = 'Hello World'    
  elif rclType == 'Int8':
    node = rclpy.create_node('py_int8_pulbisher')
    publisher = node.create_publisher(Int8, 'Int8_py_js_channel', 10)
    msg = Int8()
    msg.data = 127  
  elif rclType == 'UInt8':
    node = rclpy.create_node('py_uint8_pulbisher')
    publisher = node.create_publisher(UInt8, 'UInt8_py_js_channel', 10)
    msg = UInt8()
    msg.data = 255
  elif rclType == 'Int16':
    node = rclpy.create_node('py_int16_publisher')
    publisher = node.create_publisher(Int16, 'Int16_py_js_channel', 10)
    msg = Int16()
    msg.data = 0x7fff
  elif rclType == 'UInt16':
    node = rclpy.create_node('py_uint16_publisher')
    publisher = node.create_publisher(UInt16, 'UInt16_py_js_channel', 10)
    msg = UInt16()
    msg.data = 0xffff;
  elif rclType == 'Int32':
    node = rclpy.create_node('py_int32_publisher')
    publisher = node.create_publisher(Int32, 'Int32_py_js_channel', 10)
    msg = Int32()
    msg.data = 0x7fffffff
  elif rclType == 'UInt32':
    node = rclpy.create_node('py_uint32_publisher')
    publisher = node.create_publisher(UInt32, 'UInt32_py_js_channel', 10)
    msg = UInt32()
    msg.data = 0xffffffff
  elif rclType == 'Int64':
    node = rclpy.create_node('py_int64_publisher')
    publisher = node.create_publisher(Int64, 'Int64_py_js_channel', 10)
    msg = Int64()
    msg.data = pow(2, 53) - 1
  elif rclType == 'UInt64':
    node = rclpy.create_node('py_uint64_publisher')
    publisher = node.create_publisher(UInt64, 'UInt64_py_js_channel', 10)
    msg = UInt64()
    msg.data = pow(2, 53) - 1
  elif rclType == 'Float32':
    node = rclpy.create_node('py_float32_publisher')
    publisher = node.create_publisher(Float32, 'Float32_py_js_channel', 10)
    msg = Float32()
    msg.data = 3.14
  elif rclType == 'Float64':
    node = rclpy.create_node('py_float64_publisher')
    publisher = node.create_publisher(Float64, 'Float64_py_js_channel', 10)
    msg = Float64()
    msg.data = 3.14
  elif rclType == 'Array':
    node = rclpy.create_node('py_array_publisher')
    publisher = node.create_publisher(ByteMultiArray, 'Array_py_js_channel', 10);

    lengthDim = MultiArrayDimension()
    lengthDim.label = 'length'
    lengthDim.size = 1;
    lengthDim.stride = 3;

    layout = MultiArrayLayout()
    layout.dim = [lengthDim]
    layout.data_offset = 0;

    msg = ByteMultiArray()
    msg.layout = layout
    msg.data = [b'\x41', b'\x42', b'\x43']
  elif rclType == 'ColorRGBA':
    node = rclpy.create_node('py_colorrgba_publisher')
    publisher = node.create_publisher(ColorRGBA, 'ColorRGBA_py_js_channel', 10)
    msg = ColorRGBA()
    msg.a = 0.5
    msg.r = 127.0
    msg.g = 255.0
    msg.b = 255.0
  elif rclType == 'Header':
    node = rclpy.create_node('py_header_publisher')
    publisher = node.create_publisher(Header, 'Header_py_js_channel', 10)
    time = Time()
    time.sec = 123456
    time.nanosec = 789
    msg = Header()
    msg.stamp = time
    msg.frame_id = 'main frame'
  elif rclType == 'JointState':
    node = rclpy.create_node('py_jointstate_publisher')
    publisher = node.create_publisher(JointState, 'JointState_py_js_channel', 10)
    time = Time()
    time.sec = 123456
    time.nanosec = 789
    header = Header()
    header.stamp = time
    header.frame_id = 'main frame'
    msg = JointState()
    msg.header = header
    msg.name = ['Tom', 'Jerry']
    msg.position = [1.0, 2.0]
    msg.velocity = [2.0, 3.0]
    msg.effort = [4.0, 5.0, 6.0]
  else:
    sys.stderr.write('Unsupported type')
    sys.exit(0)
  while True:
    publisher.publish(msg)
    sleep(0.5)

  cleanup()


if __name__ == '__main__':
  main()
