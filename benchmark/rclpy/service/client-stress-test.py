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
import time

import rclpy
from rclpy.node import Node
from nav_msgs.srv import GetMap

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-r", "--run", type=int, default=1, help="How many times to run")
    args = parser.parse_args()

    rclpy.init()
    
    try:
        node = Node('stress_client_rclpy')
        
        # Create client
        client = node.create_client(GetMap, 'get_map')
        
        # Wait for service to be available
        node.get_logger().info('Waiting for service to be available...')
        if not client.wait_for_service(timeout_sec=5.0):
            node.get_logger().error('Service not available after waiting')
            return
        
        node.get_logger().info(f'The client will send a GetMap request continuously '
                              f'until receiving {args.run} response times.')
        
        start_time = time.time()
        received_times = 0
        
        # Send requests synchronously
        for i in range(args.run):
            request = GetMap.Request()
            try:
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
                
                if future.result() is not None:
                    received_times += 1
                else:
                    node.get_logger().error(f'Request {i+1} failed')
            except Exception as e:
                node.get_logger().error(f'Service call {i+1} failed: {e}')
        
        # Log completion
        diff = time.time() - start_time
        milliseconds, seconds = math.modf(diff)
        node.get_logger().info(f'Benchmark took {int(seconds)} seconds and '
                              f'{round(milliseconds * 1000)} milliseconds.')
        node.get_logger().info(f'Successfully received {received_times}/{args.run} responses')
        
        node.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
