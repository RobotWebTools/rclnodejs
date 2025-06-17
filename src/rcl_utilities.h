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

#ifndef SRC_RCL_UTILITIES_H_
#define SRC_RCL_UTILITIES_H_

#include <napi.h>
#include <rcl/graph.h>
#include <rmw/rmw.h>

#include <memory>
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

// Store a reference to the environment that can be used for error reporting.
Napi::Env& GetEnv();
void StoreEnv(Napi::Env current_env);

std::unique_ptr<rmw_qos_profile_t> GetQoSProfile(Napi::Value qos);
void ExtractNamesAndTypes(rcl_names_and_types_t names_and_types,
                          Napi::Array* result_list);

Napi::Array ConvertToJSTopicEndpointInfoList(
    Napi::Env env, const rmw_topic_endpoint_info_array_t* info_array);

Napi::Value ConvertToQoS(Napi::Env env, const rmw_qos_profile_t* qos_profile);

// `AbstractArgsFromNapiArray` and `FreeArgs` must be called in pairs.
char** AbstractArgsFromNapiArray(const Napi::Array& jsArgv);
// `AbstractArgsFromNapiArray` and `FreeArgs` must be called in pairs.
void FreeArgs(char** argv, size_t argc);

bool HasUnparsedROSArgs(const rcl_arguments_t& rcl_args);

}  // namespace rclnodejs

#endif  // SRC_RCL_UTILITIES_H_
