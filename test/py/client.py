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
import time
import rclpy
from std_msgs.msg import String
from std_msgs.msg import Int8
from example_interfaces.srv import AddTwoInts

node = None

def cleanup():
  global node
  node.destroy_node()
  rclpy.shutdown()

def main():
  global node

  service = 'py_js_add_two_ints'
  if len(sys.argv) > 1:
    service = sys.argv[1]

  rclpy.init()
  node = rclpy.create_node('add_client')
  client = node.create_client(AddTwoInts, service)
  publisher = node.create_publisher(Int8, 'back_' + service, 10)
  request = AddTwoInts.Request()
  request.a = 1
  request.b = 2

  msg = Int8()

  while not client.wait_for_service(timeout_sec=1.0):
    node.get_logger().info('service not available, waiting again...')

  future = client.call_async(request)
  rclpy.spin_until_future_complete(node, future)

  if future.result() is not None:
      msg.data = future.result().sum
      print(future.result().sum)
      publisher.publish(msg)
  time.sleep(0.1)
  cleanup()


if __name__ == '__main__':
  main()
