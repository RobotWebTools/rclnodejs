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
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "utilities.hpp"

void ShowUsage(const std::string name) {
    std::cerr << "Usage: " << name << " [options]\n"
              << "\nOptions:\n"
              << "\n--size [size_kb]\tThe block size\n"
              << "--run <n>         \tHow many times to run\n"
              << "--help            \toutput usage information"
              << std::endl;
}

int main(int argc, char* argv[]) {
  auto total_times = 0;
  auto amount = 0;

  for (int i = 1; i < argc; i++) {
      std::string arg = argv[i];
      if ((arg == "-h") || (arg == "--help")) {
          ShowUsage(argv[0]);
          return 0;
      } else if (arg.find("--size=") != std::string::npos) {
          amount = std::stoi(arg.substr(arg.find("=") + 1));
      } else if (arg.find("--run=") != std::string::npos) {
          total_times = std::stoi(arg.substr(arg.find("=") + 1));
      }
  }

  rclcpp::init(argc, argv);
  auto height_dim = std::make_shared<std_msgs::msg::MultiArrayDimension>();
  height_dim->label = "height";
  height_dim->size = 10;
  height_dim->stride = 600;

  auto width_dim = std::make_shared<std_msgs::msg::MultiArrayDimension>();
  width_dim->label = "width";
  width_dim->size = 20;
  width_dim->stride = 60;

  auto channel_dim = std::make_shared<std_msgs::msg::MultiArrayDimension>();
  channel_dim->label = "channel";
  channel_dim->size = 3;
  channel_dim->stride = 4;

  auto layout = std::make_shared<std_msgs::msg::MultiArrayLayout>();
  layout->dim = std::vector<std_msgs::msg::MultiArrayDimension>{
      *height_dim, *width_dim, *channel_dim};
  layout->data_offset = 0;

  auto msg = std::make_shared<std_msgs::msg::UInt8MultiArray>();
  msg->layout = *layout;
  msg->data = std::vector<uint8_t>(1024 * amount, 255);

  printf(
      "The publisher will publish a UInt8MultiArray topic(contains a size of "
      "%dKB array) %d times.\n", amount, total_times);

  auto start = std::chrono::high_resolution_clock::now();
  auto node = rclcpp::Node::make_shared("stress_publisher_rclcpp");
  auto publisher =
      node->create_publisher<std_msgs::msg::UInt8MultiArray>("stress_topic");
  auto sent_times = 0;

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
  }

  return 0;
}
