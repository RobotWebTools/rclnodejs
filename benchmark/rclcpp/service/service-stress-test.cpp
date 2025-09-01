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

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "nav_msgs/srv/get_map.hpp"
#include "rclcpp/rclcpp.hpp"

void ShowUsage(const std::string name) {
  std::cerr << "Usage: " << name << " [options]\n"
            << "\nOptions:\n"
            << "\n-s, --size [size_kb]\tThe block size\n"
            << "-h, --help            \toutput usage information" << std::endl;
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto amount = 1;
  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if ((arg == "-h") || (arg == "--help")) {
      ShowUsage(argv[0]);
      return 0;
    } else if (arg.find("--size=") != std::string::npos) {
      amount = std::stoi(arg.substr(arg.find("=") + 1));
    } else if (arg == "-s" && i + 1 < argc) {
      amount = std::stoi(argv[++i]);
    }
  }

  auto node = rclcpp::Node::make_shared("stress_service_rclcpp");
  auto service = node->create_service<nav_msgs::srv::GetMap>(
      "get_map",
      [amount](const std::shared_ptr<nav_msgs::srv::GetMap::Request> request,
               std::shared_ptr<nav_msgs::srv::GetMap::Response> response) {
        (void)request;
        response->map.header.stamp = rclcpp::Clock().now();
        response->map.header.frame_id = "main_frame";
        response->map.info.map_load_time = rclcpp::Clock().now();
        response->map.info.resolution = 1.0;
        response->map.info.width = 1024;
        response->map.info.height = 768;
        response->map.info.origin.position.x = 0.0;
        response->map.info.origin.position.y = 0.0;
        response->map.info.origin.position.z = 0.0;
        response->map.info.origin.orientation.x = 0.0;
        response->map.info.origin.orientation.y = 0.0;
        response->map.info.origin.orientation.z = 0.0;
        response->map.info.origin.orientation.w = 1.0;
        response->map.data = std::vector<int8_t>(1024 * amount, 125);
      });
  rclcpp::spin(node);

  return 0;
}
