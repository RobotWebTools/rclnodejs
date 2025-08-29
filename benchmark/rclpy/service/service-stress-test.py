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
from rclpy.node import Node
from rclpy.clock import Clock
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Quaternion, Pose

class StressService(Node):
    def __init__(self, size):
        super().__init__('stress_service_rclpy')
        self.size = size
        
        # Create service
        self.service = self.create_service(GetMap, 'get_map', self.handle_get_map)
        
        # Prepare map data
        self.map_data = self.create_map_data()
        
        self.get_logger().info(f'Service ready to serve GetMap requests with {size}KB data')

    def create_map_data(self):
        # Create timestamp
        clock = Clock()
        current_time = clock.now().to_msg()
        
        # Create header
        header = Header()
        header.stamp = current_time
        header.frame_id = 'main_frame'

        # Create map metadata
        info = MapMetaData()
        info.map_load_time = current_time
        info.resolution = 1.0
        info.width = 1024
        info.height = 768

        # Create origin pose
        position = Point()
        position.x = 0.0
        position.y = 0.0
        position.z = 0.0

        orientation = Quaternion()
        orientation.x = 0.0
        orientation.y = 0.0
        orientation.z = 0.0
        orientation.w = 1.0  # Valid quaternion

        origin = Pose()
        origin.position = position
        origin.orientation = orientation
        info.origin = origin

        # Create occupancy grid
        map_data = OccupancyGrid()
        map_data.header = header
        map_data.info = info
        map_data.data = [x & 0x7f for x in range(1024 * self.size)]
        
        return map_data

    def handle_get_map(self, request, response):
        response.map = self.map_data
        return response

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--size", type=int, default=1000, help="The block size[kb]")
    args = parser.parse_args()

    rclpy.init()
    
    try:
        stress_service = StressService(args.size)
        rclpy.spin(stress_service)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
