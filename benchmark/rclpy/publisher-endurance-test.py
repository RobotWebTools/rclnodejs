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
from datetime import datetime
from std_msgs.msg import *
from builtin_interfaces.msg import *
from sensor_msgs.msg import *
import threading

def main():
  rclpy.init()

  print('The publisher will publish a JointState topic every 100ms.')
  print('Begin at ' + str(datetime.now()) + 'and end in about 24 hours')
  node = rclpy.create_node('endurance_publisher_rclpy')
  publisher = node.create_publisher(JointState, 'endurance_topic')
  time = Time()
  time.sec = 123456
  time.nanosec = 789
  header = Header()
  header.stamp = time
  header.frame_id = 'main_frame'
  msg = JointState()
  msg.header = header
  msg.name = ['Tom', 'Jerry']
  msg.position = [1.0, 2.0]
  msg.velocity = [2.0, 3.0]
  msg.effort = [4.0, 5.0, 6.0]
  totalTimes = 864000
  sentTimes = 0

  def publish_topic():
    nonlocal publisher
    nonlocal msg
    nonlocal timer
    nonlocal sentTimes
    if sentTimes > totalTimes:
      timer.cancel()
      node.destroy_node()
      rclpy.shutdown()
      print('End at ' + str(datetime.now()))
    else:
      publisher.publish(msg)
      sentTimes += 1
      timer = threading.Timer(0.1, publish_topic)
      timer.start()

  timer = threading.Timer(0.1, publish_topic)
  timer.start()

if __name__ == '__main__':
  main()

