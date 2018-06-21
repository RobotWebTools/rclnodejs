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
import rclpy
from std_srvs.srv import *
from std_msgs.msg import *
from nav_msgs.srv import *
from nav_msgs.msg import *
from builtin_interfaces.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *

parser = argparse.ArgumentParser()
parser.add_argument("-s", "--size", type=int, help="The block size[kb]")
args = parser.parse_args()
if args.size is None:
  args.size = 1000

map_data = OccupancyGrid()
stamp = Time();
stamp.sec = 123456
stamp.nanosec = 789
header = Header()
header.stamp = stamp;
header.frame_id = 'main_frame'

info = MapMetaData()
map_load_time = Time()
map_load_time.sec = 123456
map_load_time.nanosec = 789
info.resolution = 1.0
info.width = 1024
info.height = 768

position = Point()
position.x = 0.0
position.y = 0.0
position.z = 0.0

orientation = Quaternion()
orientation.x = 0.0
orientation.y = 0.0
orientation.z = 0.0
orientation.w = 0.0

origin = Pose()
origin.position = position
origin.orientation = orientation
info.map_load_time = map_load_time
info.origin = origin;

map_data.header = header
map_data.info = info
map_data.data = [x & 0x7f for x in range(1024 * args.size)]

def callback(request, response):
  global map_data
  response.map = map_data
  return response

def main():
  rclpy.init()
  node = rclpy.create_node('stress_service_rclpy')
  service = node.create_service(GetMap, 'get_map', callback)

  while rclpy.ok():
    rclpy.spin_once(node)

if __name__ == '__main__':
  main()
