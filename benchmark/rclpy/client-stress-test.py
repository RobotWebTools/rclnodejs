#!/usr/bin/env python3
# Copyright (c) 2018 Intel Corporation. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from datetime import datetime
from nav_msgs.srv import *

def main():
  rclpy.init()

  print(
    'The client will send a GetMap request continuously(response contains a size of 10MB array) until receiving 36000 response times.')
  print('Begin at ' + str(datetime.now()))
  node = rclpy.create_node('stress_client_rclpy')
  client = node.create_client(GetMap, 'get_map')
  request = GetMap.Request()
  totalTimes = 36000
  receivedTimes = 0

  while rclpy.ok():
    if receivedTimes > totalTimes:
      node.destroy_node()
      rclpy.shutdown()
      print('End at ' + str(datetime.now()))
    else:
      client.call(request)
      rclpy.spin_once(node)
      if client.response is not None:
        receivedTimes += 1

if __name__ == '__main__':
  main()
