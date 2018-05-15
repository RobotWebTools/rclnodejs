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

import rclpy
from builtin_interfaces.msg import *
from datetime import datetime
from std_msgs.msg import *
import threading

def main():
  rclpy.init()

  times = input('How many times do you want to run? ')
  amount = input('The amount of data(MB) to be sent in one loop. ')
  amount = int(amount)

  widthDim = MultiArrayDimension()
  widthDim.label = 'width'
  widthDim.size = 20;
  widthDim.stride = 60;

  heightDim = MultiArrayDimension()
  heightDim.label = 'height'
  heightDim.size = 10;
  heightDim.stride = 600;

  channelDim = MultiArrayDimension()
  channelDim.label = 'channel'
  channelDim.size = 3;
  channelDim.stride = 4;

  layout = MultiArrayLayout()
  layout.dim = [widthDim, heightDim, channelDim]
  layout.data_offset = 0;

  multiArray = UInt8MultiArray()
  multiArray.layout = layout
  multiArray.data = [x & 0xff for x in range(1024 * 1024 * amount)]

  print('The publisher will publish a UInt8MultiArray topic(contains a size of %dMB array) %s times.' % (amount, times))
  print('Begin at ' + str(datetime.now()))
  node = rclpy.create_node('stress_publisher_rclpy')
  publisher = node.create_publisher(UInt8MultiArray, 'stress_topic')
  totalTimes = int(times)
  sentTimes = 0

  while rclpy.ok():
    if sentTimes > totalTimes:
      node.destroy_node()
      rclpy.shutdown()
      print('End at ' + str(datetime.now()))
    else:
      publisher.publish(multiArray)
      sentTimes += 1

if __name__ == '__main__':
  main()
