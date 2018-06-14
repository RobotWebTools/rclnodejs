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

import argparse
import math
from nav_msgs.srv import *
import rclpy
from time import time

def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("-r", "--run", type=int, help="How many times to run")
  args = parser.parse_args()
  if args.run is None:
    args.run = 1

  rclpy.init()
  print(
    'The client will send a GetMap request continuously until receiving %s response times.' % args.run)
  start = time();
  node = rclpy.create_node('stress_client_rclpy')
  client = node.create_client(GetMap, 'get_map')
  request = GetMap.Request()
  received_times = 0

  while rclpy.ok():
    if received_times > args.run:
      node.destroy_node()
      rclpy.shutdown()
      diff = time() - start
      milliseconds, seconds = math.modf(diff)
      print('Benchmark took %d seconds and %d milliseconds.' % (seconds, round(milliseconds * 1000)))
    else:
      future = client.call_async(request)
      rclpy.spin_until_future_complete(node, future)
      if future.result() is not None:
        received_times += 1

if __name__ == '__main__':
  main()
