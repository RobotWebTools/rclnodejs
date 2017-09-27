#!/usr/bin/env python3

import sys
import signal
from time import sleep

import rclpy
from std_msgs.msg import *

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
    publisher = node.create_publisher(Bool, 'Bool_py_js_channel')
    msg = Bool()
    msg.data = True
  elif rclType == 'Byte':
    node = rclpy.create_node('py_byte_publisher')
    publisher = node.create_publisher(Byte, 'Byte_py_js_channel')
    msg = Byte()
    msg.data = b'\xff'
  elif rclType == 'Char':
    node = rclpy.create_node('py_char_publisher')
    publisher = node.create_publisher(Char, 'Char_py_js_channel')
    msg = Char()
    msg.data = 'A'
  elif rclType == 'String':
    node = rclpy.create_node('py_string_publisher')
    publisher = node.create_publisher(String, 'String_py_js_channel')
    msg = String()
    msg.data = 'Hello World'    
  elif rclType == 'Int8':
    node = rclpy.create_node('py_int8_pulbisher')
    publisher = node.create_publisher(Int8, 'Int8_py_js_channel')
    msg = Int8()
    msg.data = 127  
  elif rclType == 'UInt8':
    node = rclpy.create_node('py_uint8_pulbisher')
    publisher = node.create_publisher(UInt8, 'UInt8_py_js_channel')
    msg = UInt8()
    msg.data = 255
  elif rclType == 'Int16':
    node = rclpy.create_node('py_int16_publisher')
    publisher = node.create_publisher(Int16, 'Int16_py_js_channel')
    msg = Int16()
    msg.data = 0x7fff
  elif rclType == 'UInt16':
    node = rclpy.create_node('py_uint16_publisher')
    publisher = node.create_publisher(UInt16, 'UInt16_py_js_channel')
    msg = UInt16()
    msg.data = 0xffff;
  elif rclType == 'Int32':
    node = rclpy.create_node('py_int32_publisher')
    publisher = node.create_publisher(Int32, 'Int32_py_js_channel')
    msg = Int32()
    msg.data = 0x7fffffff
  elif rclType == 'UInt32':
    node = rclpy.create_node('py_uint32_publisher')
    publisher = node.create_publisher(UInt32, 'UInt32_py_js_channel')
    msg = UInt32()
    msg.data = 0xffffffff
  elif rclType == 'Int64':
    node = rclpy.create_node('py_int64_publisher')
    publisher = node.create_publisher(Int64, 'Int64_py_js_channel')
    msg = Int64()
    msg.data = pow(2, 53) - 1
  elif rclType == 'UInt64':
    node = rclpy.create_node('py_uint64_publisher')
    publisher = node.create_publisher(UInt64, 'UInt64_py_js_channel')
    msg = UInt64()
    msg.data = pow(2, 53) - 1
  elif rclType == 'Float32':
    node = rclpy.create_node('py_float32_publisher')
    publisher = node.create_publisher(Float32, 'Float32_py_js_channel')
    msg = Float32()
    msg.data = 3.14
  elif rclType == 'Float64':
    node = rclpy.create_node('py_float64_publisher')
    publisher = node.create_publisher(Float64, 'Float64_py_js_channel')
    msg = Float64()
    msg.data = 3.14
  else:
    sys.stderr.write('Unsupported type')
    sys.exit(0)
  while True:
    print(msg.data)
    publisher.publish(msg)
    sleep(0.5)

  cleanup()


if __name__ == '__main__':
  main()
