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
from std_msgs.msg import String
from example_interfaces.srv import AddTwoInts

node = None

def cleanup():
  global node
  node.destroy_node()
  rclpy.shutdown()

def callback(request, response):
  response.sum = request.a + request.b
  return response

def main():
  global node

  service = 'js_py_add_two_ints'
  if len(sys.argv) > 1:
    service = sys.argv[1]

  rclpy.init()
  node = rclpy.create_node('add_service')
  service = node.create_service(AddTwoInts, service, callback)
  while rclpy.ok():
    rclpy.spin_once(node)

  cleanup()

if __name__ == '__main__':
  main()
