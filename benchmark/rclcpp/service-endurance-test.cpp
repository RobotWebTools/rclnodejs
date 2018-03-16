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

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("endurance_service_rclnodejs");
  auto service = node->create_service<std_srvs::srv::SetBool>(
      "set_flag",
      [](const std::shared_ptr<rmw_request_id_t> request_header,
         const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
         const std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        (void)request_header;
        (void)request;
        response->success = true;
        response->message = "The flag has been set.";
      });
  rclcpp::spin(node);

  return 0;
}
