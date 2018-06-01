// Copyright (c) 2018 Intel Corporation. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "./utilities.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto msg = std::make_shared<sensor_msgs::msg::JointState>();
  msg->header.stamp.sec = 123456;
  msg->header.stamp.nanosec = 789;
  msg->header.frame_id = std::string("main_frame");
  msg->name = std::vector<std::string>{"Tom", "Jerry"};
  msg->position = std::vector<double>{1.0, 2.0};
  msg->velocity = std::vector<double>{2.0, 3.0};
  msg->effort = std::vector<double>{4.0, 5.0, 6.0};

  auto totalTimes = 0;
  int ms = 0;
  printf("How many times do you want to run?\n");
  scanf("%d", &totalTimes);
  printf("Please enter the period of publishing a topic in millisecond\n");
  scanf("%d", &ms);

  printf(
      "The publisher will publish a JointState topic %d times every %dms\n",
      totalTimes, ms);
  auto start = std::chrono::high_resolution_clock::now();
  auto node = rclcpp::Node::make_shared("endurance_publisher_rclcpp");
  auto publisher =
      node->create_publisher<sensor_msgs::msg::JointState>("endurance_topic");
  auto sentTimes = 0;

  auto period = std::chrono::milliseconds(ms);
  rclcpp::WallRate wall_rate(period);
  while (rclcpp::ok()) {
    if (sentTimes > totalTimes) {
      rclcpp::shutdown();
      auto end = std::chrono::high_resolution_clock::now();
      LogTimeConsumption(start, end);
    } else {
      publisher->publish(msg);
      sentTimes++;
      rclcpp::spin_some(node);
    }
    wall_rate.sleep();
  }

  return 0;
}
