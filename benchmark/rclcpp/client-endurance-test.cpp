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
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "./utilities.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto totalTimes = 0;
  printf("How many times do you want to run?\n");
  scanf("%d", &totalTimes);
  printf(
      "The client will send a SetBool request every 100ms until receiving"
      " response %d times.\n", totalTimes);
  printf("Begin at %s\n", GetCurrentTime());

  auto node = rclcpp::Node::make_shared("endurance_client_rclcpp");
  auto client = node->create_client<std_srvs::srv::SetBool>("set_flag");
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  auto receivedTimes = 0;

  while (rclcpp::ok()) {
    if (receivedTimes > totalTimes) {
      rclcpp::shutdown();
      printf("End at %s\n", GetCurrentTime());
    } else {
      auto result_future = client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(node, result_future) !=
          rclcpp::executor::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "service call failed.")
        return 1;
      }
      auto result = result_future.get();
      receivedTimes++;
    }
  }
  return 0;
}
