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

#include "nav_msgs/srv/get_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "utilities.hpp"

void ShowUsage(const std::string name) {
  std::cerr << "Usage: " << name << " [options]\n"
            << "\nOptions:\n"
            << "\n-r, --run <n>     \tHow many times to run\n"
            << "-h, --help          \toutput usage information" << std::endl;
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto total_times = 0;
  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if ((arg == "-h") || (arg == "--help")) {
      ShowUsage(argv[0]);
      return 0;
    } else if (arg.find("--run=") != std::string::npos) {
      total_times = std::stoi(arg.substr(arg.find("=") + 1));
    } else if (arg == "-r" && i + 1 < argc) {
      total_times = std::stoi(argv[++i]);
    }
  }
  printf(
      "The client will send a GetMap request continuously until receiving "
      "response %d times.\n",
      total_times);

  auto start = std::chrono::high_resolution_clock::now();
  auto node = rclcpp::Node::make_shared("stress_client_rclcpp");
  auto client = node->create_client<nav_msgs::srv::GetMap>("get_map");
  auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
  auto received_times = 0;

  // Wait for service to be available
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Service not available, waiting again...");
  }

  while (rclcpp::ok()) {
    if (received_times >= total_times) {
      rclcpp::shutdown();
      auto end = std::chrono::high_resolution_clock::now();
      LogTimeConsumption(start, end);
      break;
    } else {
      auto result_future = client->async_send_request(request);

      // Spin until future is ready using the node directly
      auto future_status = rclcpp::spin_until_future_complete(
          node, result_future, std::chrono::seconds(10));

      if (future_status != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Service call failed or timed out.");
        return 1;
      }

      auto result = result_future.get();
      received_times++;
    }
  }
  return 0;
}
