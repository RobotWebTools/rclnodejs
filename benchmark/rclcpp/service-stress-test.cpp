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

#include <memory>
#include <vector>

#include "nav_msgs/srv/get_map.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto amount = 0;
  printf("The amount of data(KB) to be sent for each request.\n");
  scanf("%d", &amount);

  auto node = rclcpp::Node::make_shared("stress_service_rclcpp");
  auto sub = node->create_service<nav_msgs::srv::GetMap>(
      "get_map",
      [amount](const std::shared_ptr<rmw_request_id_t> request_header,
         const std::shared_ptr<nav_msgs::srv::GetMap::Request> request,
         const std::shared_ptr<nav_msgs::srv::GetMap::Response> response) {
        (void)request_header;
        (void)request;
        response->map.header.stamp.sec = 123456;
        response->map.header.stamp.nanosec = 789;
        response->map.header.frame_id = "main_frame";
        response->map.info.map_load_time.sec = 123456;
        response->map.info.map_load_time.nanosec = 789;
        response->map.info.resolution = 1.0;
        response->map.info.width = 1024;
        response->map.info.height = 768;
        response->map.info.origin.position.x = 0.0;
        response->map.info.origin.position.y = 0.0;
        response->map.info.origin.position.z = 0.0;
        response->map.info.origin.orientation.x = 0.0;
        response->map.info.origin.orientation.y = 0.0;
        response->map.info.origin.orientation.z = 0.0;
        response->map.info.origin.orientation.w = 0.0;
        response->map.data = std::vector<int8_t>(1024 * amount, 125);
      });
  rclcpp::spin(node);

  return 0;
}
