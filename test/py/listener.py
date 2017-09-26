#!/usr/bin/env python3

import sys
import rclpy
from time import sleep
from std_msgs.msg import String
import signal

node = None

def cleanup():
  global node
  node.destroy_node()
  rclpy.shutdown()

def handler(signum, frame):
  cleanup()
  sys.exit(0)

signal.signal(signal.SIGINT, handler)

def callback(msg):
  sys.stdout.write(msg.data)
  sys.stdout.flush()

def main():
  global node
  rclpy.init()
  node = rclpy.create_node('py_listener')
  publisher = node.create_subscription(String, 'chatter_py', callback)
  while rclpy.ok():
    rclpy.spin_once(node)

  cleanup()


if __name__ == '__main__':
  main()
