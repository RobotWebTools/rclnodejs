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
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, MultiArrayDimension, MultiArrayLayout

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--size", type=int, default=1, help="The block size[kb]")
    parser.add_argument("-r", "--run", type=int, default=1, help="How many times to run")
    args = parser.parse_args()

    rclpy.init()
    
    try:
        node = Node('stress_publisher_rclpy')
        
        # Create publisher
        publisher = node.create_publisher(UInt8MultiArray, 'stress_topic', 10)
        
        # Prepare message
        width_dim = MultiArrayDimension()
        width_dim.label = 'width'
        width_dim.size = 20
        width_dim.stride = 60

        height_dim = MultiArrayDimension()
        height_dim.label = 'height'
        height_dim.size = 10
        height_dim.stride = 600

        channel_dim = MultiArrayDimension()
        channel_dim.label = 'channel'
        channel_dim.size = 3
        channel_dim.stride = 4

        layout = MultiArrayLayout()
        layout.dim = [width_dim, height_dim, channel_dim]
        layout.data_offset = 0

        msg = UInt8MultiArray()
        msg.layout = layout
        msg.data = [x & 0xff for x in range(1024 * args.size)]
        
        node.get_logger().info(f'The publisher will publish a UInt8MultiArray topic '
                              f'(contains a size of {args.size}KB array) {args.run} times.')
        
        discovery_deadline = time.perf_counter() + 5.0
        while publisher.get_subscription_count() == 0:
            if time.perf_counter() >= discovery_deadline:
                node.get_logger().error('No matched subscriber available after 5 seconds.')
                raise RuntimeError('No matched subscriber available')
            rclpy.spin_once(node, timeout_sec=0.01)

        sent_times = 0
        start_time = time.perf_counter()
        
        # Publish messages
        for i in range(args.run):
            publisher.publish(msg)
            sent_times += 1
            rclpy.spin_once(node, timeout_sec=0.001)

        end_time = time.perf_counter()

        # Log completion
        diff = end_time - start_time
        milliseconds, seconds = math.modf(diff)
        node.get_logger().info(f'Benchmark took {int(seconds)} seconds and '
                              f'{round(milliseconds * 1000)} milliseconds.')
        node.get_logger().info(f'Successfully published {sent_times}/{args.run} messages')
        
        node.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
