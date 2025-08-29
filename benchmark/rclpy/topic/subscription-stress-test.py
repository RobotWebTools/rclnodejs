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
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

class StressSubscriber(Node):
    def __init__(self):
        super().__init__('stress_subscription_rclpy')
        self.subscription = self.create_subscription(
            UInt8MultiArray,
            'stress_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Subscription node ready to receive messages')

    def listener_callback(self, msg):
        # Just consume the message for benchmarking
        pass

def main():
    rclpy.init()
    
    try:
        stress_subscriber = StressSubscriber()
        rclpy.spin(stress_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
