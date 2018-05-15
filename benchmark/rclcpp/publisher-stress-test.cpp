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
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "./utilities.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto totalTimes = 0;
  auto amount = 0;
  printf("How many times do you want to run?\n");
  scanf("%d", &totalTimes);
  printf("The amount of data(MB) to be sent in one loop.\n");
  scanf("%d", &amount);

  auto heightDim = std::make_shared<std_msgs::msg::MultiArrayDimension>();
  heightDim->label = "height";
  heightDim->size = 10;
  heightDim->stride = 600;

  auto widthDim = std::make_shared<std_msgs::msg::MultiArrayDimension>();
  widthDim->label = "width";
  widthDim->size = 20;
  widthDim->stride = 60;

  auto channelDim = std::make_shared<std_msgs::msg::MultiArrayDimension>();
  channelDim->label = "channel";
  channelDim->size = 3;
  channelDim->stride = 4;

  auto layout = std::make_shared<std_msgs::msg::MultiArrayLayout>();
  layout->dim = std::vector<std_msgs::msg::MultiArrayDimension>{
      *heightDim, *widthDim, *channelDim};
  layout->data_offset = 0;

  auto msg = std::make_shared<std_msgs::msg::UInt8MultiArray>();
  msg->layout = *layout;
  msg->data = std::vector<uint8_t>(1024 * 1024 * amount, 255);

  printf(
      "The publisher will publish a UInt8MultiArray topic(contains a size of "
      "%dMB array) %d times.\n", amount, totalTimes);
  printf("Begin at %s\n", GetCurrentTime());

  auto node = rclcpp::Node::make_shared("stress_publisher_rclcpp");
  auto publisher =
      node->create_publisher<std_msgs::msg::UInt8MultiArray>("stress_topic");
  auto sentTimes = 0;

  while (rclcpp::ok()) {
    if (sentTimes > totalTimes) {
      rclcpp::shutdown();
      printf("End at %s\n", GetCurrentTime());
    } else {
      publisher->publish(msg);
      sentTimes++;
      rclcpp::spin_some(node);
    }
  }

  return 0;
}
