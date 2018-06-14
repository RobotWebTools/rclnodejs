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
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "./utilities.hpp"

void ShowUsage(const std::string name) {
    std::cerr << "Usage: " << name << " [options]\n"
              << "\nOptions:\n"
              << "\n--period [period_ms]\tThe period(ms) to send a topic\n"
              << "--run <n>             \tHow many times to run\n"
              << "--help                \toutput usage information"
              << std::endl;
}

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

  auto total_times = 0;
  auto ms = 0;

  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if ((arg == "-h") || (arg == "--help")) {
        ShowUsage(argv[0]);
        return 0;
    } else if (arg.find("--period=") != std::string::npos) {
        ms = std::stoi(arg.substr(arg.find("=") + 1));
    } else if (arg.find("--run=") != std::string::npos) {
        total_times = std::stoi(arg.substr(arg.find("=") + 1));
    }
  }

  printf(
      "The publisher will publish a JointState topic %d times every %dms\n",
      total_times, ms);
  auto start = std::chrono::high_resolution_clock::now();
  auto node = rclcpp::Node::make_shared("endurance_publisher_rclcpp");
  auto publisher =
      node->create_publisher<sensor_msgs::msg::JointState>("endurance_topic");
  auto sent_times = 0;

  auto period = std::chrono::milliseconds(ms);
  rclcpp::WallRate wall_rate(period);
  while (rclcpp::ok()) {
    if (sent_times > total_times) {
      rclcpp::shutdown();
      auto end = std::chrono::high_resolution_clock::now();
      LogTimeConsumption(start, end);
    } else {
      publisher->publish(msg);
      sent_times++;
      rclcpp::spin_some(node);
    }
    wall_rate.sleep();
  }

  return 0;
}
