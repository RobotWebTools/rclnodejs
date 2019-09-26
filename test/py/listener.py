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
from std_msgs.msg import String

node = None
publisher = None

def cleanup():
  global node
  node.destroy_node()
  rclpy.shutdown()

def callback(msg):
  global publisher
  publisher.publish(msg)

def main():
  global node
  global publisher

  topic = 'js_py_chatter'
  if len(sys.argv) > 1:
    topic = sys.argv[1]

  rclpy.init()
  node = rclpy.create_node('py_listener')
  publisher = node.create_publisher(String, 'back_' + topic, 10)
  subscription = node.create_subscription(String, topic, callback, 10)
  while rclpy.ok():
    rclpy.spin_once(node)

  cleanup()


if __name__ == '__main__':
  main()
