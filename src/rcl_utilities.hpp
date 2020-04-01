// Copyright (c) 2017 Intel Corporation. All rights reserved.
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

#ifndef RCLNODEJS_RCL_UTILITIES_HPP_
#define RCLNODEJS_RCL_UTILITIES_HPP_

#include <string>

struct rosidl_message_type_support_t;
struct rosidl_service_type_support_t;
struct rosidl_action_type_support_t;

namespace rclnodejs {

const rosidl_message_type_support_t* GetMessageTypeSupport(
    const std::string& package_name, const std::string& sub_folder,
    const std::string& msg_name);

const rosidl_service_type_support_t* GetServiceTypeSupport(
    const std::string& package_name, const std::string& service_name);

const rosidl_action_type_support_t* GetActionTypeSupport(
    const std::string& package_name, const std::string& action_name);

std::string GetErrorMessageAndClear();

}  // namespace rclnodejs

#endif
