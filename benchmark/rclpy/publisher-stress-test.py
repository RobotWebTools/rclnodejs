#!/usr/bin/env python3
# Copyright (c) 2017 Intel Corporation. All rights reserved.
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
import rclpy
from builtin_interfaces.msg import *
import math
from std_msgs.msg import *
import threading
from time import time

def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("-s", "--size", type=int, help="The block size[kb]")
  parser.add_argument("-r", "--run", type=int, help="How many times to run")
  args = parser.parse_args()
  rclpy.init()
  if args.size is None:
    args.size = 1
  if args.run is None:
    args.run = 1

  amount = args.size
  width_dim = MultiArrayDimension()
  width_dim.label = 'width'
  width_dim.size = 20;
  width_dim.stride = 60;

  height_dim = MultiArrayDimension()
  height_dim.label = 'height'
  height_dim.size = 10;
  height_dim.stride = 600;

  channel_dim = MultiArrayDimension()
  channel_dim.label = 'channel'
  channel_dim.size = 3;
  channel_dim.stride = 4;

  layout = MultiArrayLayout()
  layout.dim = [width_dim, height_dim, channel_dim]
  layout.data_offset = 0;

  msg = UInt8MultiArray()
  msg.layout = layout
  msg.data = [x & 0xff for x in range(1024 * amount)]

  print('The publisher will publish a UInt8MultiArray topic(contains a size of %dKB array) %s times.' % (amount, args.run))
  start = time()
  node = rclpy.create_node('stress_publisher_rclpy')
  publisher = node.create_publisher(UInt8MultiArray, 'stress_topic')
  total_times = args.run
  sent_times = 0

  while rclpy.ok():
    if sent_times > total_times:
      node.destroy_node()
      rclpy.shutdown()
      diff = time() - start
      milliseconds, seconds = math.modf(diff)
      print('Benchmark took %d seconds and %d milliseconds.' % (seconds, round(milliseconds * 1000)))
    else:
      publisher.publish(msg)
      sent_times += 1

if __name__ == '__main__':
  main()
