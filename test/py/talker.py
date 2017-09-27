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
import signal
from time import sleep
from std_msgs.msg import String

import rclpy

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
  signal.signal(signal.SIGINT, handler)

  rclpy.init()
  node = rclpy.create_node('py_talker')
  publisher = node.create_publisher(String, 'py_js_chatter')

  msg = String()
  msg.data = 'Hello World'
  
  while True:
    print(msg.data)
    publisher.publish(msg)
    sleep(0.1)

  cleanup()


if __name__ == '__main__':
  main()
